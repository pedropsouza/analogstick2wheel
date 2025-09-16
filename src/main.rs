use input_linux::*;
use std::{
    convert::From, default::Default, hash::{DefaultHasher, Hash, Hasher}, io::{self, Read, Write}, mem, ops::{Deref, SubAssign}, time::{Duration, Instant, UNIX_EPOCH}
};
use timeout_readwrite::TimeoutReader;

#[derive(Clone, Copy, Hash)]
struct Frame {
    x: i32,
    y: i32,
    state: State,
}

impl Frame {
    pub fn analyze(&self) -> (f64,f64,State) {
        let x = self.x as f64;
        let y = self.y as f64;
        let mag = (x.powi(2) + y.powi(2)).sqrt()/MAX_MAGNITUDE;
        (y.atan2(x), mag, if mag > GRIP_THRESHOLD { State::Gripped } else { State::Freewheel })
    }

    pub fn resolve(&mut self) -> (f64, f64) {
        let (a,m,s) = self.analyze();
        self.state = s;
        (a,m)
    }
}

impl Default for Frame {
    fn default() -> Self {
        Self {
            x: Default::default(),
            y: Default::default(),
            state: State::Freewheel,
        }
    }
}

#[derive(Clone, Default)]
struct ProcessedFrame {
    inner: Frame,
    analog_angle: Option<f64>,
    analog_magnitude: f64,
}

impl From<Frame> for ProcessedFrame {
    fn from(mut value: Frame) -> Self {
        let (a,m) = value.resolve();
        Self {
            inner: value,
            analog_angle: Some(a),
            analog_magnitude: m,
        }
    }
}

impl Deref for ProcessedFrame {
    type Target = Frame;
    fn deref(&self) -> &Self::Target {
        &self.inner
    }
}

impl ProcessedFrame {
    pub fn dbg_string(&self) -> String {
        format!("x: {: >8.6}, y: {: >8.6}, analog_angle: {}, analog_magnitude: {: >8.6}, state: {:?}",
                self.inner.x, self.inner.y,
                self.analog_angle.map_or(" Undef. ".to_string(), |opt| format!("{: >8.6}", opt.to_degrees())),
                self.analog_magnitude, self.state,
        )
    }
}

#[derive(Clone, Copy, PartialEq, Eq, Debug, Hash)]
enum State {
    Freewheel,
    Gripped,
}

// symmetrical, 5/4 ratio comes from the 900deg sweep, 450deg to each side
//const STEERING_STOP: f64 = std::f64::consts::TAU * 5.0/4.0;
const STEERING_STOP: f64 = std::f64::consts::TAU * 3.0;
const MAX_MAGNITUDE: f64 = 128.0;
const GRIP_THRESHOLD: f64 = 0.92;

struct Data {
    prev: ProcessedFrame,
    cur: Frame,
    wheel_angle: f64,
    last_wheel_report: Instant,
}

fn main() {
    let mut timeout_stdin = TimeoutReader::new(io::stdin(), Duration::from_millis(20));
    let mut data = Data {
        last_wheel_report: Instant::now(),
        wheel_angle: 0.0,
        prev: Default::default(),
        cur: Default::default(),
    };

    let tick = |data: &mut Data, event: SynchronizeEvent| {
        if event.kind == SynchronizeKind::Report {
            let processed = ProcessedFrame::from(data.cur);
            data.wheel_angle = wheel_behaviour(data.wheel_angle, &processed, &data.prev, data.last_wheel_report.elapsed().as_secs_f64());
            let axis_val = quantize_wheel_angle(data.wheel_angle);
            write_output_event(axis_val, event.time);
            eprintln!("{}, wheel_angle: {: >8.6} aka {:>5}   ",
                    processed.dbg_string(),
                    data.wheel_angle.to_degrees(),
                    axis_val);
            Some(processed)
        } else {
            io::stdout().write_all(event.as_event().as_bytes()).unwrap();
            None
        }
    };

    loop {
        #[derive(Debug)]
        enum LoopError {
            Timeout,
            EventCreation(RangeError),
            Other(String),
        }
        let input =
            read_input_event(&mut timeout_stdin)
            .map_err(|e| match e {
                e if e.kind() == io::ErrorKind::TimedOut => LoopError::Timeout,
                e => LoopError::Other(e.to_string())
            })
            .and_then(|x| { Event::new(x).map_err(|e| LoopError::EventCreation(e)) });

        match input {
            Ok(event) => {
                match event {
                    Event::Absolute(event) => {
                        match event.axis {
                            AbsoluteAxis::X => { data.cur.x = event.value - 128 },
                            AbsoluteAxis::Y => { data.cur.y = event.value - 128 },
                            _ => {
                                io::stdout().write_all(event.as_event().as_bytes()).unwrap();
                            },
                        }
                    },
                    Event::Synchronize(event) => {
                        if let Some(processed) = tick(&mut data, event) {
                            data.prev = processed;
                            data.last_wheel_report = Instant::now();
                        }
                        let sepoch = UNIX_EPOCH.elapsed().unwrap();
                        let skew = sepoch.saturating_sub(
                            Duration::from_secs(event.time.seconds() as u64).saturating_add(Duration::from_micros(event.time.microseconds() as u64))
                        );

                        eprintln!("skew is {:?}", skew);
                    },
                    _ => {
                        io::stdout().write_all(event.as_event().as_bytes()).unwrap()
                    }
                }
            },
            Err(e) => {
                match e {
                    LoopError::Timeout => {},
                    LoopError::Other(oe) => {
                        eprintln!("error: {oe}");
                        // flush garbage out
                        let mut buf: [u8; 256] = [0; 256];
                        let _ = io::stdin().read(&mut buf);
                    }
                    LoopError::EventCreation(ce) => {
                        eprintln!("value range error: {ce}")
                    }
                }
            }
        }
        if data.wheel_angle.abs() > 0.0005 && data.prev.state == State::Freewheel {
            let delta = Instant::now().duration_since(data.last_wheel_report);
            if delta > Duration::from_millis(2) {
                let unix_time = UNIX_EPOCH.elapsed().unwrap();
                let timestamp = EventTime::new(unix_time.as_secs() as i64, unix_time.subsec_micros() as i64);
                tick(&mut data, SynchronizeEvent::report(timestamp));
                data.last_wheel_report = Instant::now();
            }
        }
        io::stdout().flush().unwrap();
    }
}

fn lerp(from: f64, to: f64, t: f64) -> f64 {
    let t = t.clamp(0.0,1.0);
    return (1.0-t)*from + t*to;
}

fn wheel_behaviour(cur_wheel_angle: f64, cur: &ProcessedFrame, prev: &ProcessedFrame, d_t: f64) -> f64 {
    let easing = || { lerp(cur_wheel_angle, 0.0, ((std::f64::consts::TAU/4.0)*d_t).clamp(0.0,0.2)) };
    cur.analog_angle.map(|aangle| {
        match (prev.state, cur.state) {
            (State::Gripped, State::Gripped) => {
                let da = prev.analog_angle.map_or(0.0, |p_aangle| {
                    cyclic_signed_distance(aangle, p_aangle)
                });
                da + cur_wheel_angle
            },
            _ => easing()
        }
    }).unwrap_or_else(easing).clamp(-STEERING_STOP, STEERING_STOP)
}

fn cyclic_signed_distance(a: f64, b: f64) -> f64 {
    let mut r = a - b;
    const T: f64 = std::f64::consts::TAU;
    const P: f64 = std::f64::consts::PI;
    loop {
        if r > -P { break; }
        r += T;
    }
    loop {
        if r < P { break; }
        r -= T;
    }
    r
}

fn quantize_wheel_angle(angle: f64) -> i32 {
    const HALF_U16: i32 = u16::MAX as i32/2;
    HALF_U16 + (HALF_U16 as f64/STEERING_STOP * angle).trunc() as i32
}

fn read_input_event<T: Read>(handle: &mut T) -> io::Result<InputEvent> {
    let mut buffer = [0u8; mem::size_of::<InputEvent>()];
    handle.read_exact(&mut buffer)?;
    let event = unsafe { mem::transmute(buffer) };
    return Ok(event)
}

fn write_output_event(axis_value: i32, timestamp: EventTime) {
    let synthesized_event
        = AbsoluteEvent::new(
            timestamp,
            AbsoluteAxis::X,
            axis_value,
        );
    let output: Vec<u8>
        = [
            synthesized_event.into_event(),
            SynchronizeEvent::new(timestamp, SynchronizeKind::Report, 0).into_event(),
        ].iter()
         .flat_map(|x| x.into_bytes())
         .collect();
    io::stdout().write_all(&output).unwrap();
}
