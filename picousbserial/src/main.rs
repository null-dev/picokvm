//! # Pico USB Serial Example
//!
//! Creates a USB Serial device on a Pico board, with the USB driver running in
//! the main thread.
//!
//! This will create a USB Serial device echoing anything it receives. Incoming
//! ASCII characters are converted to upercase, so you can tell it is working
//! and not just local-echo!
//!
//! See the `Cargo.toml` file for Copyright and license details.

#![no_std]
#![no_main]

// The macro for our start-up function
use rp_pico::entry;

// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use panic_halt as _;

// A shorter alias for the Peripheral Access Crate, which provides low-level
// register access
use rp_pico::hal::{Clock, pac};

// A shorter alias for the Hardware Abstraction Layer, which provides
// higher-level drivers.
use rp_pico::hal;

// USB Device support
use usb_device::{class_prelude::*, prelude::*};

// USB Communications Class Device support
use usbd_serial::SerialPort;

/// Import the GPIO pins we use
use hal::gpio::pin::bank0::{Gpio0, Gpio1};

// Time handling traits
use fugit::RateExtU32;

// Used to demonstrate writing formatted strings
use fugit::TimerDurationU64;
use rp_pico::hal::timer::Instant;

use embedded_hal::digital::v2::{InputPin, OutputPin};
use rp_pico::hal::uart::{DataBits, StopBits, UartConfig};

/// Alias the type for our UART pins to make things clearer.
type UartPins = (
    hal::gpio::Pin<Gpio0, hal::gpio::Function<hal::gpio::Uart>>,
    hal::gpio::Pin<Gpio1, hal::gpio::Function<hal::gpio::Uart>>,
);

/// Alias the type for our UART to make things clearer.
type Uart = hal::uart::UartPeripheral<hal::uart::Enabled, pac::UART0, UartPins>;

const BLINK_DURATION: TimerDurationU64<1_000_000> = TimerDurationU64::millis(10);

/// Entry point to our bare-metal application.
///
/// The `#[entry]` macro ensures the Cortex-M start-up code calls this function
/// as soon as all global variables are initialised.
///
/// The function configures the RP2040 peripherals, then echoes any characters
/// received over USB Serial.
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
    // let mut resync_pin = pins.gpio2.into_pull_down_input(); // RESYNC ONLY
    // let mut resync_resp_pin = pins.gpio3.into_push_pull_output(); // RESYNC ONLY
    let mut led_pin = pins.led.into_push_pull_output();

    let uart_pins = (
        // UART TX (characters sent from RP2040) on pin 1 (GPIO0)
        pins.gpio0.into_mode::<hal::gpio::FunctionUart>(),
        // UART RX (characters received by RP2040) on pin 2 (GPIO1)
        pins.gpio1.into_mode::<hal::gpio::FunctionUart>(),
    );
    // Make a UART on the given pins
    let uart = hal::uart::UartPeripheral::new(pac.UART0, uart_pins, &mut pac.RESETS)
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

    // Set up the USB Communications Class Device driver
    let mut serial = SerialPort::new(&usb_bus);

    // Create a USB device with a fake VID and PID
    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x1209, 0x0003))
        .manufacturer("VEPTA")
        .product("evserial converter")
        .serial_number("42069")
        .device_class(2) // from: https://www.usb.org/defined-class-codes
        .build();

    let timer = hal::Timer::new(pac.TIMER, &mut pac.RESETS);
    let mut last_data: Instant = timer.get_counter();
    let mut blink_high: bool = false;
    /*let mut ignore_resync: bool = false;
    let mut first_resync_requested: bool = false; */ // RESYNC ONLY
    loop {
        let counter = timer.get_counter();

        // Check for new data
        if usb_dev.poll(&mut [&mut serial]) {
            let mut buf = [0u8; 64];
            match serial.read(&mut buf) {
                Err(_e) => {
                    // TODO
                }
                Ok(0) => {
                    // TODO
                }
                Ok(count) => {
                    last_data = counter;
                    let wr_ptr = &buf[..count];
                    uart.write_full_blocking(wr_ptr);
                    // Send back to the host
                    /*let mut wr_ptr = &buf[..count];
                    while !wr_ptr.is_empty() {
                        match serial.write(wr_ptr) {
                            Ok(len) => wr_ptr = &wr_ptr[len..],
                            // On error, just drop unwritten data.
                            // One possible error is Err(WouldBlock), meaning the USB
                            // write buffer is full.
                            Err(_) => break,
                        };
                    }*/
                }
            }
        }

        /*if resync_pin.is_high() == Ok(true)  {
            if !ignore_resync {
                // Highly unlikely this will block as we don't send much data this direction
                let _ = serial.write(&[1]);
                resync_resp_pin.set_high();
                ignore_resync = true;
            }
        } else if ignore_resync {
            ignore_resync = false;
            resync_resp_pin.set_low();
        }

        // Immediately request resync
        if !first_resync_requested {
            let _ = serial.write(&[1]);
            first_resync_requested = true;
        }*/ // RESYNC ONLY

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
    }
}