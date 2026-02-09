//! Pure Rust library for communicating with FTDI USB devices.
//!
//! This crate provides a complete interface to FTDI USB-to-serial converter
//! chips, including the FT232R, FT2232H, FT4232H, FT232H, and FT230X families.
//! It uses [nusb](https://crates.io/crates/nusb) as the USB backend — no
//! C dependencies or `libusb` required.
//!
//! # Quick Start
//!
//! ```no_run
//! use ftdi::{FtdiDevice, constants::FTDI_VID, constants::pid};
//!
//! // Open the first FT232R connected
//! let mut dev = FtdiDevice::open(FTDI_VID, pid::FT232)?;
//! dev.set_baudrate(115200)?;
//! dev.write_all(b"Hello from Rust!\r\n")?;
//! # Ok::<(), ftdi::Error>(())
//! ```
//!
//! # Features
//!
//! - **Device discovery**: Enumerate and filter connected FTDI devices.
//! - **Serial I/O**: Baud rate, line properties, flow control, modem lines.
//! - **Bitbang / MPSSE**: Asynchronous and synchronous bitbang, MPSSE for
//!   SPI/I2C/JTAG.
//! - **High-level MPSSE**: SPI master and I2C master with typed APIs
//!   ([`mpsse::spi`], [`mpsse::i2c`]).
//! - **Async transfers**: Submit non-blocking USB reads/writes and wait
//!   for completion later ([`async_transfer`]).
//! - **EEPROM**: Read, write, erase, decode, and build EEPROM images with
//!   chip-aware defaults.
//! - **Streaming**: High-throughput continuous reads via concurrent USB
//!   transfers (FT2232H / FT232H).
//! - **`Read` / `Write` traits**: Use `FtdiDevice` anywhere `std::io::Read`
//!   or `std::io::Write` is expected.

pub mod async_transfer;
mod baudrate;
pub mod constants;
pub mod context;
pub mod device_info;
pub mod eeprom;
pub mod error;
#[cfg(feature = "embedded-hal")]
pub mod hal;
pub mod mpsse;
pub mod stream;
pub mod types;

// ---- Convenience re-exports ----

pub use async_transfer::{ReadTransferControl, WriteTransferControl};
pub use constants::FTDI_VID;
pub use context::FtdiDevice;
pub use device_info::{find_device, find_devices, DeviceFilter};
pub use eeprom::FtdiEeprom;
pub use error::{Error, Result};
pub use stream::StreamProgress;
pub use types::*;
