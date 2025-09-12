use input_linux::*;
use std::{default, hash::{DefaultHasher, Hash, Hasher}, io::{self, Read, Write}, mem};
use ordered_float::OrderedFloat;

// make floats hashable
type F64 = OrderedFloat<f64>;

#[derive(Clone, Copy, Hash)]
struct Frame {
    x: F64,
    y: F64,
    joystick_angle: F64,
    wheel_angle: F64,
    state: State,
}

#[derive(Clone, Copy, PartialEq, Eq, Debug, Hash)]
enum State {
    Freewheel,
    Gripped,
    Sliding,
}

impl default::Default for Frame {
    fn default() -> Self {
        Self {
            x: Default::default(),
            y: Default::default(),
            joystick_angle: Default::default(),
            wheel_angle: Default::default(),
            state: State::Freewheel,
        }
    }
}

impl Frame {
    pub fn analyze(&mut self) -> f64 {
        self.joystick_angle = F64::from(self.y.atan2(*self.x));
        let mag = self.x.powi(2) + self.y.powi(2);
        self.state = if mag > GRIP_THRESHOLD { State::Gripped } else { State::Freewheel };
        mag
    }

    pub fn dbg_string(&self) -> String {
        format!("x: {: >8.6}, y: {: >8.6}, joy_angle: {: >8.6}, wheel_angle: {: >8.6}, state: {:?}",
                self.x, self.y, self.joystick_angle, self.wheel_angle, self.state,
        )
    }

    pub fn quickhash(&self) -> u64 {
        let mut s = DefaultHasher::new();
        self.hash(&mut s);
        s.finish()
    }
}

// symmetrical, 5/4 ratio comes from the 900deg sweep, 450deg to each side
//const STEERING_STOP: f64 = std::f64::consts::TAU * 5.0/4.0;
const STEERING_STOP: f64 = std::f64::consts::TAU * 2.0;
const GRIP_THRESHOLD: f64 = 1.0/5.0 * 1.0/5.0;
const SLIDE_THRESHOLD: f64 = 9.5/10.0 * 9.5/10.0;

fn main() {
    let mut frames = [Frame::default()].repeat(2);
    let mut prev_hash = frames[1].quickhash();

    loop {
        let input =
            read_input_event()
            .map_err(|e| format!("{e}"))
            .and_then(|x| { Event::new(x).map_err(|e| format!("{e}")) });
        if let Err(e) = input {
            eprintln!("error: {e}");
            // flush garbage out
            let mut buf: [u8; 256] = [0; 256];
            let _ = io::stdin().read(&mut buf);
            continue;
        }
        let event = input.unwrap();

        match event {
            Event::Absolute(event) => {
                match event.axis {
                    AbsoluteAxis::X => { frames[0].x = F64::from((event.value - 128) as f64)/128.0 },
                    AbsoluteAxis::Y => { frames[0].y = F64::from((event.value - 128) as f64)/128.0 },
                    _ => {
                        io::stdout().write_all(event.as_event().as_bytes()).unwrap()
                    },
                }
            },
            Event::Synchronize(event) => {
                let magnitude = frames[0].analyze();
                let cur_hash = frames[0].quickhash();
                let diff = prev_hash != cur_hash;
                if event.kind == SynchronizeKind::Report && diff {
                    let axis_val = wheel_behaviour(frames.as_mut_slice(), magnitude);
                    frames[1] = frames[0];
                    prev_hash = cur_hash;
                    let synthesized_event
                        = AbsoluteEvent::new(
                            event.time,
                            AbsoluteAxis::X,
                            axis_val,
                        );
                    let output: Vec<u8>
                        = [
                            synthesized_event.into_event(),
                            event.into_event()
                        ].iter()
                         .flat_map(|x| x.into_bytes())
                         .collect();
                    io::stdout().write_all(&output).unwrap();
                    eprint!("\r{}                     ", frames[0].dbg_string())
                } else {
                    io::stdout().write_all(event.as_event().as_bytes()).unwrap()
                }
            }
            _ => {
                io::stdout().write_all(event.as_event().as_bytes()).unwrap()
            }
        }
        io::stdout().flush().unwrap();
    }
}

fn lerp(from: f64, to: f64, t: f64) -> f64 {
    return (1.0-t)*from + t*to;
}

fn wheel_behaviour(frames: &mut [Frame], magnitude: f64) -> i32 {
    let mut prev = frames[1];
    let cur = &mut frames[0];
    let mut angle = None;

    match cur.state {
        State::Gripped => {
            if prev.state == State::Gripped {
                if magnitude < SLIDE_THRESHOLD {
                    cur.state = State::Sliding;
                    angle = Some(lerp(*cur.wheel_angle, 0.0, 0.01)); // this is horrible for many reasons.
                }
            } else {
                prev = *cur;
            }
        },
        State::Sliding => unreachable!(),
        State::Freewheel => { prev = *cur }
    }

    let angle = angle.unwrap_or({
        let da = F64::from(cyclic_signed_distance(cur.joystick_angle.into(), prev.joystick_angle.into()));
        cur.wheel_angle = F64::from((*da + *prev.wheel_angle).clamp(-STEERING_STOP, STEERING_STOP));
        *cur.wheel_angle
    });

    let quantized_value = 512 + ((512.0/STEERING_STOP * angle).trunc() as i32);
    quantized_value
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

fn read_input_event() -> io::Result<InputEvent> {
    let mut buffer = [0u8; mem::size_of::<InputEvent>()];
    io::stdin().read_exact(&mut buffer)?;
    let event = unsafe { mem::transmute(buffer) };
    return Ok(event)
}
