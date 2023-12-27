# picokvm
 
Keyboard/mouse switch using two Raspberry Pi Picos (you only need one of you already have a USB to serial adapter)
 
## Subdirectories
 
- **evserial**: Daemon that forwards Linux evdev events to USB serial when toggled with an activation key combination.
- **picousbserial**: Firmware for Raspberry Pi Pico. Makeshift USB to serial adapter.
- **picousbhid**: Firmware for Raspberry Pi Pico. Receives evdev events over serial and translates them into USB HID events.
 
## Project state
 
I created this project mainly for personal use. Expect bugs and little maintenance.

## Known bugs:

- evserial doesn't reconnect properly when USB serial is disconnected and reconnected
