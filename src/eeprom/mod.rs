//! FTDI EEPROM support: reading, writing, encoding, and decoding.
//!
//! The EEPROM on FTDI chips stores device identification, USB descriptor
//! strings, pin configuration, and other settings. This module provides:
//!
//! - [`FtdiEeprom`] - The decoded EEPROM structure.
//! - [`build`] - Encode the structure into a binary image.
//! - [`decode`] - Decode a binary image into the structure.
//! - I/O operations on [`FtdiDevice`](crate::FtdiDevice) for reading/writing
//!   the physical EEPROM.

pub mod build;
pub mod decode;
mod io;
mod types;

pub use types::FtdiEeprom;
