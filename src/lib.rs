#![no_std]

use core::cell::RefCell;

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::blocking_mutex::Mutex as BlockingMutex;
use esp_hal::i2c::master::I2c;
use esp_hal::Blocking;

pub mod led;
pub mod network;
pub mod sensors;

/// Shared blocking I2C bus type used by all sensor tasks.
pub type I2cBusBlocking = BlockingMutex<CriticalSectionRawMutex, RefCell<I2c<'static, Blocking>>>;
