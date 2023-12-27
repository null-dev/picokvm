//! # Pico USB 'Twitchy' Mouse Example
//!
//! Creates a USB HID Class Pointing device (i.e. a virtual mouse) on a Pico
//! board, with the USB driver running in the main thread.
//!
//! It generates movement reports which will twitch the cursor up and down by a
//! few pixels, several times a second.
//!
//! See the `Cargo.toml` file for Copyright and license details.
//!
//! This is a port of
//! https://github.com/atsamd-rs/atsamd/blob/master/boards/itsybitsy_m0/examples/twitching_usb_mouse.rs

#![no_std]
#![no_main]

mod evdev;
mod keyboard;

use cortex_m::prelude::*;
// The macro for our start-up function
use rp_pico::entry;

// The macro for marking our interrupt functions
use rp_pico::hal::pac::interrupt;

// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use panic_halt as _;

// Pull in any important traits
use rp_pico::hal::prelude::*;

// A shorter alias for the Peripheral Access Crate, which provides low-level
// register access
use rp_pico::hal::pac;

// A shorter alias for the Hardware Abstraction Layer, which provides
// higher-level drivers.
use rp_pico::hal;
use rp_pico::hal::uart::{DataBits, StopBits, UartConfig};

// USB Device support
use usb_device::{class_prelude::*, prelude::*};

// Time handling traits
use fugit::{ExtU32, MicrosDuration, MicrosDurationU64, MillisDurationU32, RateExtU32, TimerDurationU32, TimerDurationU64};

/// Import the GPIO pins we use
use hal::gpio::pin::bank0::{Gpio0, Gpio1};
use rp_pico::hal::timer::Instant;

use embedded_hal::digital::v2::{InputPin, OutputPin, StatefulOutputPin};
use usbd_human_interface_device::prelude::*;
use crate::evdev::{CheckedEvdevEvent, EvdevEvent, InputEventKind, Key, RelativeAxisType};
use frunk::HList;
use usbd_human_interface_device::device::consumer::ConsumerControlInterface;
use usbd_human_interface_device::device::keyboard::{NKROBootKeyboardInterface, NKROBootKeyboardReport};
use usbd_human_interface_device::device::mouse::{WheelMouseInterface, WheelMouseReport};
use usbd_human_interface_device::interface::raw::RawInterfaceConfig;
use usbd_human_interface_device::interface::WrappedInterfaceConfig;
use usbd_human_interface_device::page::Keyboard;
use usbd_serial::SerialPort;
use crate::keyboard::map_key;

/// The USB Device Driver (shared with the interrupt).
static mut USB_DEVICE: Option<UsbDevice<hal::usb::UsbBus>> = None;

/// The USB Bus Driver (shared with the interrupt).
static mut USB_BUS: Option<UsbBusAllocator<hal::usb::UsbBus>> = None;

/// The USB Human Interface Device Driver (shared with the interrupt).
static mut COMPOSITE_HID: Option<HIDComposite> = None;

/// Serial device (shared with interrupt)
// SERIAL DEBUG
// static mut SERIAL_DEV: Option<SerialPort<hal::usb::UsbBus>> = None;

/// Alias the type for our UART pins to make things clearer.
type UartPins = (
    hal::gpio::Pin<Gpio0, hal::gpio::Function<hal::gpio::Uart>>,
    hal::gpio::Pin<Gpio1, hal::gpio::Function<hal::gpio::Uart>>,
);

/// Alias the type for our UART to make things clearer.
type Uart = hal::uart::UartPeripheral<hal::uart::Enabled, pac::UART0, UartPins>;

type HIDComposite = UsbHidClass<
    hal::usb::UsbBus,
    HList!(
        ConsumerControlInterface<'static, hal::usb::UsbBus>,
        WheelMouseInterface<'static, hal::usb::UsbBus>,
        NKROBootKeyboardInterface<'static, hal::usb::UsbBus>,
    ),
>;

const BLINK_DURATION: TimerDurationU64<1_000_000> = TimerDurationU64::millis(10);
// Time we wait after suspend before allowing wake up
const WAKE_UP_GRACE_PERIOD: TimerDurationU64<1_000_000> = TimerDurationU64::secs(5);
// const BLINK_DURATION: TimerDurationU64<1_000_000> = TimerDurationU64::millis(100);
// const RESYNC_BYTE_COUNT: u16 = 10; // RESYNC ONLY
const COBS_SENTINEL: u8 = 0;
const HID_IDLE_POLL_FREQ: MicrosDurationU64 = MicrosDurationU64::millis(1);

/// Entry point to our bare-metal application.
///
/// The `#[entry]` macro ensures the Cortex-M start-up code calls this function
/// as soon as all global variables are initialised.
///
/// The function configures the RP2040 peripherals, then submits cursor movement
/// updates periodically.
#[entry]
fn main() -> ! {
    // Grab our singleton objects
    let mut pac = pac::Peripherals::take().unwrap();

    // Set up the watchdog driver - needed by the clock setup code
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    // Configure the clocks
    //
    // The default is to generate a 125 MHz system clock
    let clocks = hal::clocks::init_clocks_and_plls(
        rp_pico::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
        .ok()
        .unwrap();

    // Normally only enabled if USB workaround is needed but we need pins anyways
    // #[cfg(feature = "rp2040-e5")]
    // {
    let sio = hal::Sio::new(pac.SIO);
    let pins = rp_pico::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );
    // }
    /*let mut resync_pin = pins.gpio2.into_push_pull_output();
    let mut resync_resp_pin = pins.gpio3.into_pull_down_input(); */ // RESYNC ONLY
    let mut led_pin = pins.led.into_push_pull_output();

    let uart_pins = (
        // UART TX (characters sent from RP2040) on pin 1 (GPIO0)
        pins.gpio0.into_mode::<hal::gpio::FunctionUart>(),
        // UART RX (characters received by RP2040) on pin 2 (GPIO1)
        pins.gpio1.into_mode::<hal::gpio::FunctionUart>(),
    );
    // Make a UART on the given pins
    let mut uart = hal::uart::UartPeripheral::new(pac.UART0, uart_pins, &mut pac.RESETS)
        .enable(
            UartConfig::new(921600.Hz(), DataBits::Eight, None, StopBits::One),
            clocks.peripheral_clock.freq(),
        )
        .unwrap();

    // Set up the USB driver
    let usb_bus = UsbBusAllocator::new(hal::usb::UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));
    unsafe {
        // Note (safety): This is safe as interrupts haven't been started yet
        USB_BUS = Some(usb_bus);
    }

    // Grab a reference to the USB Bus allocator. We are promising to the
    // compiler not to take mutable access to this global variable whilst this
    // reference exists!
    let bus_ref = unsafe { USB_BUS.as_ref().unwrap() };

    // Set up the USB HID Class Device drivers
    let mouse_config = {
        let mut c = WheelMouseInterface::default_config();
        c.inner_config.in_endpoint.poll_interval = 1;
        c
    };
    let composite = UsbHidClassBuilder::new()
        .add_interface(NKROBootKeyboardInterface::default_config())
        .add_interface(mouse_config)
        .add_interface(ConsumerControlInterface::default_config())
        //Build
        .build(bus_ref);
    unsafe {
        // Note (safety): This is safe as interrupts haven't been started yet.
        COMPOSITE_HID = Some(composite);
    }

    // SERIAL DEBUG
    /*let serial = SerialPort::new(bus_ref);
    unsafe {
        SERIAL_DEV = Some(serial);
    }*/

    // Create a USB device with a fake VID and PID
    let usb_dev = UsbDeviceBuilder::new(bus_ref, UsbVidPid(0x1209, 0x0004))
        .manufacturer("VEPTA")
        .product("evserial slave")
        .serial_number("42069")
        .device_class(0)
        .supports_remote_wakeup(true)
        .build();
    unsafe {
        // Note (safety): This is safe as interrupts haven't been started yet
        USB_DEVICE = Some(usb_dev);
    }

    let core = pac::CorePeripherals::take().unwrap();
    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());
    let timer = hal::Timer::new(pac.TIMER, &mut pac.RESETS);

    unsafe {
        // Enable the USB interrupt
        pac::NVIC::unmask(hal::pac::Interrupt::USBCTRL_IRQ);
    };

    /*
    // Wait 1 second to allow the other microcontroller to boot
    let _ = led_pin.set_high();
    delay.delay_ms(1000);
    let _ = led_pin.set_low();

    // Instruct relay to re-sync
    resync_pin.set_high().unwrap();
    // Wait for response
    while resync_resp_pin.is_low().unwrap() {
        delay.delay_ms(10);
    }
    // Resync acknowledged, stop signaling resync
    resync_pin.set_low().unwrap();*/ // RESYNC ONLY

    let mut last_data: Instant = timer.get_counter();
    let mut suspend_time: Option<Instant> = None;
    let mut blink_high: bool = false;

    /*let mut resync_seq: u16 = RESYNC_BYTE_COUNT;
    let mut cmd_buffer_idx: usize = 0;
    let mut cmd_buffer: [u8; CMD_SIZE] = [0; CMD_SIZE];
    let mut reset_next = false;*/ // RESYNC ONLY
    let mut cmd_buffer: heapless::Vec<u8, 20> = heapless::Vec::new();
    let mut skip_packet = false;

    let mut state = InputState::default();

    // We also use this to do remote wakeup
    let mut hid_idle_count_down = timer.count_down();
    hid_idle_count_down.start(1.millis());

    loop {
        let counter = timer.get_counter();

        if let Ok(b) = uart.read() {
            if !skip_packet && cmd_buffer.push(b).is_err() {
                // We overflowed the buffer, this most definitely isn't a valid command
                // Don't bother processing it
                skip_packet = true;
            }

            if b == COBS_SENTINEL {
                if !skip_packet {
                    // Process
                    if let Ok(deserialized) = postcard::from_bytes_cobs::<CheckedEvdevEvent>(&mut cmd_buffer) {
                        if let Ok(event) = deserialized.verify() {
                            last_data = counter;
                            process_command(&mut state, &event);
                        }
                    }
                }

                skip_packet = false;
                cmd_buffer.clear();
            }

            /*if resync_seq > 0 {
                if b == 0xFF {
                    resync_seq -= 1;
                } else {
                    resync_seq = RESYNC_BYTE_COUNT
                }
            } else if resync_seq == 0 {
                if b == 1 {
                    reset_next = true;
                }
                resync_seq = RESYNC_BYTE_COUNT
            }

            if !reset_next {
                cmd_buffer[cmd_buffer_idx] = b;
                if cmd_buffer_idx == cmd_buffer.len() - 1 {
                    cmd_buffer_idx = 0;

                    // First two bytes in command will never be 0xFF
                    if !(cmd_buffer[0] == 0xFF && cmd_buffer[1] == 0xFF) {
                        last_data = counter;

                        let archived: &ArchivedEvdevEvent = unsafe {
                            rkyv::archived_root::<EvdevEvent>(&cmd_buffer)
                        };
                        let deserialized = <ArchivedEvdevEvent as Deserialize<EvdevEvent, rkyv::Infallible>>::deserialize(
                            archived, &mut rkyv::Infallible
                        ).unwrap();

                        process_command(&mut state, &deserialized);
                    }

                    // SERIAL DEBUG
                    /*critical_section::with(|_| unsafe {
                        SERIAL_DEV.as_mut().map(|s| {
                            let mut to_write = cmd_buffer.len();
                            while to_write > 0 {
                                if let Ok(w) = s.write(&cmd_buffer[cmd_buffer.len() - to_write..]) {
                                    to_write -= w;
                                }
                            }
                        });
                    });*/
                } else {
                    cmd_buffer_idx += 1;
                }
            } else {
                reset_next = false;
                cmd_buffer_idx = 0;
            }*/
        }

        let mouse_report = state.mouse.report();
        let key_report = state.keyboard.report();
        if mouse_report.is_some() || key_report.is_some() {
            if let Some(suspend_time) = suspend_time {
                if let Some(since) = counter.checked_duration_since(suspend_time) {
                    if since >= WAKE_UP_GRACE_PERIOD {
                        remote_wakeup();
                    }
                }
            }

            if let Some(report) = mouse_report {
                if push_mouse_movement(report).is_ok() {
                    state.mouse.flush();
                }
            }
            if let Some(report) = key_report {
                if push_key_events(report).is_ok() {
                    state.keyboard.flush();
                }
            }
        }

        // Serial activity LED
        if let Some(since) = counter.checked_duration_since(last_data) {
            let new_blink = since < BLINK_DURATION;
            if new_blink != blink_high {
                if new_blink {
                    let _ = led_pin.set_high();
                } else {
                    let _ = led_pin.set_low();
                }
                blink_high = new_blink;
            }
        }

        if hid_idle_count_down.wait().is_ok() {
            let dev_suspended = critical_section::with(|_| unsafe {
                tick_all_managed(); // Tick HID
                USB_DEVICE.as_mut().map(|d| d.state() == UsbDeviceState::Suspend).unwrap()
            });

            // Use this chance to update our knowledge of whether or not the device is suspended
            match suspend_time {
                Some(_) => {
                    if !dev_suspended {
                        suspend_time = None
                    }
                },
                None => {
                    if dev_suspended {
                        suspend_time = Some(counter)
                    }
                }
            }

            // timer is Periodic, no need to start new count down
        }
    }
}

#[derive(Default, Clone, Eq, PartialEq)]
pub struct InputState {
    mouse: MouseState,
    keyboard: KeyboardState
}

pub trait Reportable<T> {
    fn flush(&mut self);
    fn report(&self) -> Option<T>;
}

#[derive(Default, Clone, Eq, PartialEq)]
pub struct MouseState {
    last_buttons: u8,
    buttons: u8,
    pub x: i8,
    pub y: i8,
    pub vertical_wheel: i8,
    pub horizontal_wheel: i8,
}

impl MouseState {
    pub fn set_button(&mut self, button: usize, state: bool) {
        let bit = 0x1u8 << button;
        if state {
            self.buttons |= bit;
        } else {
            self.buttons &= 0xFF - bit;
        }
    }
}

impl Reportable<WheelMouseReport> for MouseState {
    fn flush(&mut self) {
        self.last_buttons = self.buttons;
        self.x = 0;
        self.y = 0;
        self.vertical_wheel = 0;
        self.horizontal_wheel = 0;
    }

    fn report(&self) -> Option<WheelMouseReport> {
        if self.x != 0
            || self.y != 0
            || self.vertical_wheel != 0
            || self.horizontal_wheel != 0
            || self.buttons != self.last_buttons {
            Some(WheelMouseReport {
                buttons: self.buttons,
                x: self.x,
                y: self.y,
                vertical_wheel: self.vertical_wheel,
                horizontal_wheel: self.horizontal_wheel,
            })
        } else { None }
    }
}

#[derive(Default, Clone, Eq, PartialEq)]
struct KeyboardState {
    last_keys: KeySet,
    keys: KeySet
}

impl KeyboardState {
    pub fn set_key(&mut self, key: Keyboard, state: bool) {
        let existing_idx = self.keys.iter().position(|k| *k == key);
        if state {
            if existing_idx.is_none() {
                if let Some(first_avail) = self.keys.iter().position(|k| *k == Keyboard::NoEventIndicated) {
                    self.keys[first_avail] = key;
                }
            }
        } else {
            if let Some(existing_idx) = existing_idx {
                self.keys[existing_idx] = Keyboard::NoEventIndicated;
            }
        }
    }
}

impl Reportable<KeySet> for KeyboardState {
    fn flush(&mut self) {
        self.last_keys = self.keys;
    }

    fn report(&self) -> Option<KeySet> {
        if self.keys != self.last_keys {
            Some(self.keys)
        } else { None }
    }
}

fn process_command(state: &mut InputState, cmd: &EvdevEvent) -> Result<(), UsbHidError> {
    let value = cmd.value();

    match cmd.kind() {
        InputEventKind::Synchronization(_) => {
            // TODO ?
        }
        InputEventKind::Key(k) => {
            map_key(state, k, value);

            // TODO Warn if key not mapped
        },
        InputEventKind::RelAxis(t) => {
            let mouse = &mut state.mouse;
            match t {
                RelativeAxisType::REL_X => {
                    mouse.x = mouse.x.saturating_add(cmd.value() as i8);
                },
                RelativeAxisType::REL_Y => {
                    mouse.y = mouse.y.saturating_add(cmd.value() as i8);
                },
                RelativeAxisType::REL_WHEEL => {
                    mouse.vertical_wheel = mouse.vertical_wheel.saturating_add(cmd.value().clamp(-1, 1) as i8);
                },
                RelativeAxisType::REL_HWHEEL => {
                    mouse.horizontal_wheel = mouse.horizontal_wheel.saturating_add(cmd.value().clamp(-1, 1) as i8);
                },
                // TODO Support high res scroll
                // https://www.kernel.org/doc/html/latest/input/event-codes.html?highlight=wheel_hi_res#ev-rel
                // https://github.com/torvalds/linux/blob/9c1bec9c0b08abeac72ed6214b723adc224013bf/drivers/hid/hid-input.c#L246
                // RelativeAxisType::REL_WHEEL_HI_RES => {
                //     mouse.vertical_wheel = mouse.vertical_wheel.saturating_add(cmd.value().clamp(-120, 120) as i8);
                // },
                _ => return Ok(())
            }
        }
        InputEventKind::AbsAxis(_) => {}
        InputEventKind::Misc(_) => {}
        InputEventKind::Switch(_) => {}
        InputEventKind::Led(_) => {}
        InputEventKind::Sound(_) => {}
        InputEventKind::ForceFeedback(_) => {}
        InputEventKind::ForceFeedbackStatus(_) => {}
        InputEventKind::UInput(_) => {}
        InputEventKind::Other => {}
    }

    Ok(())
}

/// Submit a new mouse movement report to the USB stack.
///
/// We do this with interrupts disabled, to avoid a race hazard with the USB IRQ.
fn push_mouse_movement(report: WheelMouseReport) -> Result<(), UsbHidError> {
    critical_section::with(|_| unsafe {
        // Now interrupts are disabled, grab the global variable and, if
        // available, send it a HID report
        COMPOSITE_HID.as_mut().map(|hid| hid.interface::<WheelMouseInterface<'_, _>, _>().write_report(&report))
    }).unwrap()
}

/// Submit a new key event to the USB stack.
///
/// We do this with interrupts disabled, to avoid a race hazard with the USB IRQ.
type KeySet = [Keyboard; 20];
fn push_key_events(report: KeySet) -> Result<(), UsbHidError> {
    critical_section::with(|_| unsafe {
        // Now interrupts are disabled, grab the global variable and, if
        // available, send it a HID report
        COMPOSITE_HID.as_mut().map(|hid| hid.interface::<NKROBootKeyboardInterface<'_, _>, _>().write_report(report))
    }).unwrap()
}

/// Tell the USB bus to wake up the host device
///
/// We do this with interrupts disabled, to avoid a race hazard with the USB IRQ.
fn remote_wakeup() {
    critical_section::with(|_| unsafe {
        let device = USB_DEVICE.as_mut().unwrap();
        if device.remote_wakeup_enabled() && device.state() == UsbDeviceState::Suspend {
            device.bus().remote_wakeup()
        }
    })
}

/// We must "tick()" managed devices every 1 ms
/// Must be called from within critical section
unsafe fn tick_all_managed() -> Result<(), UsbHidError> {
    // Now interrupts are disabled, grab the global variable and, if
    // available, send it a HID report
    COMPOSITE_HID.as_mut().map(|hid| hid.interface::<NKROBootKeyboardInterface<'_, _>, _>().tick()).unwrap()
}

/// This function is called whenever the USB Hardware generates an Interrupt
/// Request.
#[allow(non_snake_case)]
#[interrupt]
unsafe fn USBCTRL_IRQ() {
    // Handle USB request
    let usb_dev = USB_DEVICE.as_mut().unwrap();
    let composite_hid = COMPOSITE_HID.as_mut().unwrap();
    // let serial = SERIAL_DEV.as_mut().unwrap(); // SERIAL DEBUG
    // if usb_dev.poll(&mut [serial]) { // SERIAL DEBUG
    if usb_dev.poll(&mut [composite_hid]) {
        // Report must be read, otherwise interrupt will not clear
        let keyboard = composite_hid.interface::<NKROBootKeyboardInterface<'_, _>, _>();
        match keyboard.read_report() {
            Err(UsbError::WouldBlock) => {}
            Err(e) => {
                core::panic!("Failed to read keyboard report: {:?}", e)
            }
            Ok(leds) => unsafe {
                /*LED_PIN
                    .as_mut()
                    .map(|p| p.set_state(PinState::from(leds.num_lock)).ok());*/
            }
        }

        // SERIAL DEBUG
        /*let mut buf = [0u8; 64];
        match serial.read(&mut buf) {
            Err(_e) => {}
            Ok(0) => {}
            Ok(count) => {
                // Ignore incoming data from USB serial
                // let wr_ptr = &buf[..count];
            }
        }*/
    }
}