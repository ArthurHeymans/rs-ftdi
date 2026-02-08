//! Error types for the ftdi crate.

use crate::types::ChipType;

/// The error type for FTDI operations.
#[derive(Debug, thiserror::Error)]
pub enum Error {
    /// An error from the nusb USB layer.
    #[error("USB error: {0}")]
    Usb(#[from] nusb::Error),

    /// A USB transfer error.
    #[error("USB transfer error: {0}")]
    Transfer(#[from] nusb::transfer::TransferError),

    /// No matching device was found.
    #[error("device not found")]
    DeviceNotFound,

    /// The USB device is unavailable (not opened or disconnected).
    #[error("USB device unavailable")]
    DeviceUnavailable,

    /// Invalid argument(s) were provided.
    #[error("invalid argument: {0}")]
    InvalidArgument(&'static str),

    /// The requested baud rate cannot be achieved within tolerance.
    #[error("unsupported baud rate: requested {requested}, nearest achievable {actual}")]
    UnsupportedBaudRate {
        /// The requested baud rate.
        requested: u32,
        /// The nearest achievable baud rate.
        actual: u32,
    },

    /// An EEPROM-related error.
    #[error("EEPROM error: {0}")]
    Eeprom(String),

    /// EEPROM checksum verification failed.
    #[error("EEPROM checksum mismatch")]
    EepromChecksum,

    /// EEPROM size exceeded by custom strings.
    #[error("EEPROM size exceeded by custom strings")]
    EepromSizeExceeded,

    /// The operation is not supported for this chip type.
    #[error("unsupported operation for chip type {0:?}")]
    UnsupportedChip(ChipType),

    /// Could not set USB configuration.
    #[error("unable to set USB configuration; make sure the default FTDI driver is not in use")]
    Configuration,

    /// Could not claim the USB interface.
    #[error("unable to claim USB device; make sure the default FTDI driver is not in use")]
    ClaimFailed,

    /// A write operation completed with zero bytes transferred.
    #[error("write returned zero bytes")]
    WriteZero,

    /// USB reset failed.
    #[error("USB reset failed")]
    ResetFailed,

    /// An I2C operation received a NACK from the slave device.
    #[error("I2C NACK: {0}")]
    I2cNack(&'static str),

    /// A descriptor read failed.
    #[error("descriptor error: {0}")]
    Descriptor(#[from] nusb::GetDescriptorError),
}

/// A specialized `Result` type for FTDI operations.
pub type Result<T> = std::result::Result<T, Error>;
