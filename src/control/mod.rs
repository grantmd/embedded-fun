use core::sync::atomic::AtomicU8;

pub const MODE_FAILSAFE: u8 = 0;
pub const MODE_MANUAL: u8 = 1;
pub const MODE_AUTONOMOUS: u8 = 2;

pub static VEHICLE_MODE: AtomicU8 = AtomicU8::new(MODE_FAILSAFE);
