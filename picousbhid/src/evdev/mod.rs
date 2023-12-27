#[macro_use]
mod attribute_set;

mod constants;
mod compat;
mod scancodes;

pub use constants::*;
pub use scancodes::*;

use core::fmt;
use crc::Crc;
use serde::Deserialize;

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum InputEventKind {
    Synchronization(Synchronization),
    Key(Key),
    RelAxis(RelativeAxisType),
    AbsAxis(AbsoluteAxisType),
    Misc(MiscType),
    Switch(SwitchType),
    Led(LedType),
    Sound(SoundType),
    ForceFeedback(u16),
    ForceFeedbackStatus(u16),
    UInput(u16),
    Other,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct EnumParseError(());

#[derive(Deserialize)]
pub struct EvdevEvent {
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

#[derive(Deserialize, Debug)]
pub struct CheckedEvdevEvent {
    event: EvdevEvent,
    hash: u8
}

impl CheckedEvdevEvent {
    pub fn verify(self) -> Result<EvdevEvent, &'static str> {
        if self.event.hash() == self.hash {
            Ok(self.event)
        } else {
            Err("CRC does not match!")
        }
    }
}

impl EvdevEvent {
    /// Returns the timestamp associated with the event.
    // #[inline]
    // pub fn timestamp(&self) -> SystemTime {
    //     timeval_to_systime(&self.0.time)
    // }

    /// Returns the type of event this describes, e.g. Key, Switch, etc.
    #[inline]
    pub fn event_type(&self) -> EventType {
        EventType(self.event_type)
    }

    /// Returns the raw "code" field directly from input_event.
    #[inline]
    pub fn code(&self) -> u16 {
        self.event_code
    }

    /// A convenience function to return `self.code()` wrapped in a certain newtype determined by
    /// the type of this event.
    ///
    /// This is useful if you want to match events by specific key codes or axes. Note that this
    /// does not capture the event value, just the type and code.
    #[inline]
    pub fn kind(&self) -> InputEventKind {
        let code = self.code();
        match self.event_type() {
            EventType::SYNCHRONIZATION => InputEventKind::Synchronization(Synchronization(code)),
            EventType::KEY => InputEventKind::Key(Key::new(code)),
            EventType::RELATIVE => InputEventKind::RelAxis(RelativeAxisType(code)),
            EventType::ABSOLUTE => InputEventKind::AbsAxis(AbsoluteAxisType(code)),
            EventType::MISC => InputEventKind::Misc(MiscType(code)),
            EventType::SWITCH => InputEventKind::Switch(SwitchType(code)),
            EventType::LED => InputEventKind::Led(LedType(code)),
            EventType::SOUND => InputEventKind::Sound(SoundType(code)),
            EventType::FORCEFEEDBACK => InputEventKind::ForceFeedback(code),
            EventType::FORCEFEEDBACKSTATUS => InputEventKind::ForceFeedbackStatus(code),
            EventType::UINPUT => InputEventKind::UInput(code),
            _ => InputEventKind::Other,
        }
    }

    /// Returns the raw "value" field directly from input_event.
    ///
    /// For keys and switches the values 0 and 1 map to pressed and not pressed respectively.
    /// For axes, the values depend on the hardware and driver implementation.
    #[inline]
    pub fn value(&self) -> i32 {
        self.value
    }
}

impl fmt::Debug for EvdevEvent {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        let mut debug = f.debug_struct("EvdevEvent");
        // debug.field("time", &self.timestamp());
        let kind = self.kind();
        if let InputEventKind::Other = kind {
            debug
                .field("type", &self.event_type())
                .field("code", &self.code());
        } else {
            debug.field("kind", &kind);
        }
        debug.field("value", &self.value()).finish()
    }
}
