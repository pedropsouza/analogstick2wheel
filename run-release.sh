#!/bin/sh
set -euC
if [ ! -e true-ev-stream.pipe ]; then
    mkfifo true-ev-stream.pipe;
fi
DEV=/dev/input/by-id/usb-Sony_Interactive_Entertainment_Wireless_Controller_Wireless_Controller-if03-event-joystick
intercept -g "$DEV" > true-ev-stream.pipe &
stdbuf -i 0 -o 0 -- ./target/release/analogstick2wheel < true-ev-stream.pipe | uinput -c desc.yaml
