use evdev::{Device, EventType, InputEventKind, Key};
use serialport::{DataBits, FlowControl, Parity, SerialPort, StopBits};
use std::io::{self, Read, Write};
use std::process::exit;
use std::sync::{Arc, Mutex};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::thread;
use std::time::{Duration, Instant, SystemTime};
use crc::Crc;
use serde::Serialize;

#[derive(Serialize, Debug)]
struct EvdevEvent {
    event_type: u16,
    event_code: u16,
    value: i32,
}

// https://users.ece.cmu.edu/~koopman/crc/index.html
const EVENT_CRC: Crc<u8> = Crc::<u8>::new(&crc::CRC_8_BLUETOOTH);

impl EvdevEvent {
    fn hash(&self) -> u8 {
        let mut digest = EVENT_CRC.digest();
        digest.update(&self.event_type.to_le_bytes());
        digest.update(&self.event_code.to_le_bytes());
        digest.update(&self.value.to_le_bytes());
        digest.finalize()
    }
}

#[derive(Serialize, Debug)]
struct CheckedEvdevEvent {
    event: EvdevEvent,
    hash: u8
}

impl CheckedEvdevEvent {
    fn new(event: EvdevEvent) -> Self {
        let hash = event.hash();
        CheckedEvdevEvent {
            event,
            hash
        }
    }
}

// const RESYNC_BYTE_COUNT: i16 = 10; // RESYNC ONLY
const GRAB_KEY: Key = Key::KEY_CALC;

fn main() {
    // Change this to your evdev device path
    let device_path = "/dev/input/by-id/full-keyboard";

    // Change this to your Raspberry Pi Pico's serial port
    let pico_serial_port = "/dev/ttyACM0";

    let Ok(mut device) = Device::open(device_path) else {
        println!("Failed to open evdev device.");
        exit(1);
    };

    let mut builder = serialport::new(pico_serial_port, 921600);
    builder = builder
        .data_bits(DataBits::Eight)
        .flow_control(FlowControl::None)
        .parity(Parity::None)
        .stop_bits(StopBits::One)
        .timeout(Duration::from_millis(1000));

    let Ok(serial) = builder.open() else {
        println!("Failed to open serial device.");
        exit(1);
    };

    // let mut serial_read = serial.try_clone().unwrap();
    let serial_write = Arc::new(Mutex::new(serial));

    // Resync algo (no longer needed, we use COBS now) ===
    /*let run_mutex = Arc::new(Mutex::new(false));
    let run_mutex_clone = run_mutex.clone();
    let run_mutex_lck = run_mutex.lock().unwrap();
    {
        let serial_write_handle = serial_write.clone();
        thread::spawn(move || {
            let mut x = [0];
            let mut first_resync_done = false;
            loop {
                // Try to lock the run mutex. This will only be successful if the subroutine is no
                // longer running, which means we should terminate as well.
                if run_mutex_clone.try_lock().is_ok() {
                    println!("Read thread dying!");
                    return;
                }

                if !first_resync_done || (serial_read.read_exact(&mut x).is_ok() && x[0] == 1) {
                    println!("Resyncing...");
                    // Re-sync
                    // Wait 1s, then write 10 zeros, followed by a single 1
                    let mut serial_write = serial_write_handle.lock().unwrap();
                    thread::sleep(Duration::from_secs(1));
                    let mut to_write = [0xFF; RESYNC_BYTE_COUNT as usize + 2];
                    to_write[0] = 2;
                    to_write[11] = 1;
                    serial_write.write_all(&to_write);
                    println!("Resync complete.");
                    first_resync_done = true;
                }
            }
        });
    }*/
    // ===

    println!("Listening on device: {}", device.name().unwrap_or_default());

    // DEBUG
    /*let mut row: Vec<String> = Vec::new();

    let mut ladder: u8 = 0;
    let mut ladder_next = || {
        (0 .. 8).map(|_| {
            let before = ladder;
            ladder = ladder.wrapping_add(1);
            before
        }).collect::<Vec<u8>>()
    };*/

    // Latency test
    /*let begin_time = SystemTime::now();
    let written = Arc::new(AtomicU64::new(0));
    let written_clone = written.clone();
    thread::spawn(move || {
        let device_path = "/dev/input/by-id/usb-VEPTA_evserial_slave_42069-if01-event-mouse";

        let Ok(mut device) = Device::open(device_path) else {
            println!("Failed to open evdev device, restarting...");
            return;
        };

        println!("Latency test booted!");

        loop {
            for event in device.fetch_events().unwrap() {
                if let InputEventKind::RelAxis(_) = event.kind() {
                    let diff = SystemTime::now().duration_since(begin_time).unwrap().as_micros() as u64;
                    println!("{}", diff - written_clone.load(Ordering::Relaxed));
                }
            }
        }
    });*/

    {
        // Move the lock into this scope. When we return from this scope, the lock will unlock.
        // Only needed for resync algo
        // let _run_mutex_lck_moved = run_mutex_lck;

        let mut grabbed = false;
        loop {
            let mut toggle_grabbed = false;
            for event in device.fetch_events().unwrap() {
                // Do not send sync events for now
                if event.event_type() == EventType::SYNCHRONIZATION {
                    continue
                }

                if let InputEventKind::Key(k) = event.kind() {
                    // Do not send key repeats
                    if event.value() == 2 {
                        continue
                    }

                    if k == GRAB_KEY && event.value() == 1 {
                        toggle_grabbed = !toggle_grabbed;
                        continue;
                    }
                }

                // Latency test
                /*if let InputEventKind::RelAxis(_) = event.kind() {
                    written.store(SystemTime::now().duration_since(begin_time).unwrap().as_micros() as u64, Ordering::Relaxed);
                }*/

                if grabbed {
                    let evdev_event = EvdevEvent {
                        event_type: event.event_type().0,
                        event_code: event.code(),
                        value: event.value(),
                    };

                    let bytes = postcard::to_allocvec_cobs(&CheckedEvdevEvent::new(evdev_event)).unwrap();
                    // let bytes = ladder_next(); // DEBUG

                    if let Ok(mut serial_write) = serial_write.try_lock() {
                        if serial_write.write_all(&bytes).is_err() {
                            println!("Serial write failure, aborting...");
                            exit(1);
                        }

                        // println!("Wrote: {}", bytes.len());

                        // DEBUG
                        /*if resync_done.load(Ordering::Relaxed) {
                            row.push(bytes.iter().map(|b| format!("{b:02x}")).collect::<Vec<_>>().join(" "));
                            if row.len() == 4 {
                                println!("{}", row.join("  "));
                                row.clear();
                            }
                        }*/
                    }
                }
            }

            if toggle_grabbed {
                let new_grabbed = !grabbed;
                let grab_change_res = if new_grabbed {
                    device.grab()
                } else {
                    device.ungrab()
                };
                if grab_change_res.is_ok() {
                    grabbed = new_grabbed;
                }
            }
        }
    }
}
