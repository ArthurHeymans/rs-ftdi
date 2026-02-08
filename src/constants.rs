//! Protocol constants for FTDI chip communication.
//!
//! These constants define the USB vendor request codes, modem control values,
//! and other wire-level details of the FTDI protocol. Most users should not
//! need to use these directly.

// ---- FTDI Vendor ID and known Product IDs ----

/// Default FTDI vendor ID.
pub const FTDI_VID: u16 = 0x0403;

/// Known FTDI product IDs.
pub mod pid {
    /// FT232AM, FT232BM, FT232R.
    pub const FT232: u16 = 0x6001;
    /// FT2232C/D/H.
    pub const FT2232: u16 = 0x6010;
    /// FT4232H.
    pub const FT4232: u16 = 0x6011;
    /// FT232H.
    pub const FT232H: u16 = 0x6014;
    /// FT230X.
    pub const FT230X: u16 = 0x6015;
}

// ---- SIO vendor request codes ----

/// Reset the port.
pub(crate) const SIO_RESET_REQUEST: u8 = 0x00;
/// Set the modem control register.
pub(crate) const SIO_SET_MODEM_CTRL_REQUEST: u8 = 0x01;
/// Set flow control register.
pub(crate) const SIO_SET_FLOW_CTRL_REQUEST: u8 = 0x02;
/// Set baud rate.
pub(crate) const SIO_SET_BAUDRATE_REQUEST: u8 = 0x03;
/// Set data characteristics (bits, parity, stop, break).
pub(crate) const SIO_SET_DATA_REQUEST: u8 = 0x04;
/// Poll modem status.
pub(crate) const SIO_POLL_MODEM_STATUS_REQUEST: u8 = 0x05;
/// Set event character.
pub(crate) const SIO_SET_EVENT_CHAR_REQUEST: u8 = 0x06;
/// Set error character.
pub(crate) const SIO_SET_ERROR_CHAR_REQUEST: u8 = 0x07;
/// Set latency timer.
pub(crate) const SIO_SET_LATENCY_TIMER_REQUEST: u8 = 0x09;
/// Get latency timer.
pub(crate) const SIO_GET_LATENCY_TIMER_REQUEST: u8 = 0x0A;
/// Set bitbang mode.
pub(crate) const SIO_SET_BITMODE_REQUEST: u8 = 0x0B;
/// Read pin states directly.
pub(crate) const SIO_READ_PINS_REQUEST: u8 = 0x0C;
/// Read EEPROM.
pub(crate) const SIO_READ_EEPROM_REQUEST: u8 = 0x90;
/// Write EEPROM.
pub(crate) const SIO_WRITE_EEPROM_REQUEST: u8 = 0x91;
/// Erase EEPROM.
pub(crate) const SIO_ERASE_EEPROM_REQUEST: u8 = 0x92;

// ---- Reset sub-commands ----

/// SIO reset (device reset).
pub(crate) const SIO_RESET_SIO: u16 = 0;
/// Flush RX FIFO (chip -> host direction).
pub(crate) const SIO_TCIFLUSH: u16 = 2;
/// Flush TX FIFO (host -> chip direction).
pub(crate) const SIO_TCOFLUSH: u16 = 1;

// ---- Flow control values ----

/// Disable flow control.
pub(crate) const SIO_DISABLE_FLOW_CTRL: u16 = 0x0;
/// RTS/CTS hardware flow control.
pub(crate) const SIO_RTS_CTS_HS: u16 = 0x1 << 8;
/// DTR/DSR hardware flow control.
pub(crate) const SIO_DTR_DSR_HS: u16 = 0x2 << 8;
/// XON/XOFF software flow control.
pub(crate) const SIO_XON_XOFF_HS: u16 = 0x4 << 8;

// ---- Modem control line values ----

/// Set DTR high.
pub(crate) const SIO_SET_DTR_HIGH: u16 = 1 | (0x1 << 8);
/// Set DTR low.
pub(crate) const SIO_SET_DTR_LOW: u16 = 0x1 << 8;
/// Set RTS high.
pub(crate) const SIO_SET_RTS_HIGH: u16 = 2 | (0x2 << 8);
/// Set RTS low.
pub(crate) const SIO_SET_RTS_LOW: u16 = 0x2 << 8;

// ---- Clock constants for baud rate calculation ----

/// H-type clock: 120 MHz.
pub(crate) const H_CLK: u32 = 120_000_000;
/// Standard clock: 48 MHz.
pub(crate) const C_CLK: u32 = 48_000_000;

// ---- EEPROM constants ----

/// Maximum EEPROM size in bytes (256 for 93xx66).
pub const FTDI_MAX_EEPROM_SIZE: usize = 256;
/// Max power is stored as value * 2 mA.
pub(crate) const MAX_POWER_MILLIAMP_PER_UNIT: u16 = 2;

// ---- MPSSE commands ----

/// MPSSE shifting and pin commands, exposed for users building MPSSE command sequences.
pub mod mpsse {
    // Shifting commands
    /// Write TDI/DO on negative TCK/SK edge.
    pub const WRITE_NEG: u8 = 0x01;
    /// Write bits, not bytes.
    pub const BITMODE: u8 = 0x02;
    /// Sample TDO/DI on negative TCK/SK edge.
    pub const READ_NEG: u8 = 0x04;
    /// LSB first.
    pub const LSB: u8 = 0x08;
    /// Write TDI/DO.
    pub const DO_WRITE: u8 = 0x10;
    /// Read TDO/DI.
    pub const DO_READ: u8 = 0x20;
    /// Write TMS/CS.
    pub const WRITE_TMS: u8 = 0x40;

    // Pin commands
    /// Set data bits low byte.
    pub const SET_BITS_LOW: u8 = 0x80;
    /// Get data bits low byte.
    pub const GET_BITS_LOW: u8 = 0x81;
    /// Set data bits high byte.
    pub const SET_BITS_HIGH: u8 = 0x82;
    /// Get data bits high byte.
    pub const GET_BITS_HIGH: u8 = 0x83;
    /// Enable loopback.
    pub const LOOPBACK_START: u8 = 0x84;
    /// Disable loopback.
    pub const LOOPBACK_END: u8 = 0x85;
    /// Set TCK divisor.
    pub const TCK_DIVISOR: u8 = 0x86;

    // H-type specific commands
    /// Disable divide-by-5 prescaler (use 60 MHz master clock).
    pub const DIS_DIV_5: u8 = 0x8A;
    /// Enable divide-by-5 prescaler (use 12 MHz master clock).
    pub const EN_DIV_5: u8 = 0x8B;
    /// Enable 3-phase data clocking.
    pub const EN_3_PHASE: u8 = 0x8C;
    /// Disable 3-phase data clocking.
    pub const DIS_3_PHASE: u8 = 0x8D;
    /// Clock bits without data transfer.
    pub const CLK_BITS: u8 = 0x8E;
    /// Clock bytes without data transfer.
    pub const CLK_BYTES: u8 = 0x8F;
    /// Clock until GPIOL1 is high.
    pub const CLK_WAIT_HIGH: u8 = 0x94;
    /// Clock until GPIOL1 is low.
    pub const CLK_WAIT_LOW: u8 = 0x95;
    /// Enable adaptive clocking.
    pub const EN_ADAPTIVE: u8 = 0x96;
    /// Disable adaptive clocking.
    pub const DIS_ADAPTIVE: u8 = 0x97;
    /// Clock bytes until GPIOL1 is high.
    pub const CLK_BYTES_OR_HIGH: u8 = 0x9C;
    /// Clock bytes until GPIOL1 is low.
    pub const CLK_BYTES_OR_LOW: u8 = 0x9D;
    /// Drive open-collector outputs (FT232H only).
    pub const DRIVE_OPEN_COLLECTOR: u8 = 0x9E;

    // Commands shared between MPSSE and Host Emulation modes
    /// Send immediate (flush buffer).
    pub const SEND_IMMEDIATE: u8 = 0x87;
    /// Wait until GPIOL1 is high.
    pub const WAIT_ON_HIGH: u8 = 0x88;
    /// Wait until GPIOL1 is low.
    pub const WAIT_ON_LOW: u8 = 0x89;

    // Host Emulation mode commands
    /// Read short address.
    pub const READ_SHORT: u8 = 0x90;
    /// Read extended address.
    pub const READ_EXTENDED: u8 = 0x91;
    /// Write short address.
    pub const WRITE_SHORT: u8 = 0x92;
    /// Write extended address.
    pub const WRITE_EXTENDED: u8 = 0x93;

    /// Calculate the TCK divisor value for a target frequency.
    ///
    /// Rate = 6,000,000 / (1 + value) for 12 MHz clock,
    /// or 30,000,000 / (1 + value) for 60 MHz clock (H-type with DIS_DIV_5).
    #[inline]
    pub const fn div_value(rate: u32) -> u16 {
        if rate == 0 {
            0xFFFF
        } else if rate > 6_000_000 {
            0
        } else if 6_000_000 / rate - 1 > 0xFFFF {
            0xFFFF
        } else {
            (6_000_000 / rate - 1) as u16
        }
    }
}

// ---- CBUS pin function enumerations ----

/// CBUS pin functions for FT232R.
pub mod cbus {
    /// TX Data Enable.
    pub const TXDEN: u8 = 0;
    /// Power Enable.
    pub const PWREN: u8 = 1;
    /// RX LED (active low).
    pub const RXLED: u8 = 2;
    /// TX LED (active low).
    pub const TXLED: u8 = 3;
    /// TX/RX LED (active low).
    pub const TXRXLED: u8 = 4;
    /// Sleep.
    pub const SLEEP: u8 = 5;
    /// 48 MHz clock output.
    pub const CLK48: u8 = 6;
    /// 24 MHz clock output.
    pub const CLK24: u8 = 7;
    /// 12 MHz clock output.
    pub const CLK12: u8 = 8;
    /// 6 MHz clock output.
    pub const CLK6: u8 = 9;
    /// IO mode for CBUS bitbang.
    pub const IOMODE: u8 = 0x0A;
    /// Bitbang write.
    pub const BB_WR: u8 = 0x0B;
    /// Bitbang read.
    pub const BB_RD: u8 = 0x0C;
}

/// CBUS pin functions for FT232H.
pub mod cbush {
    /// Tristate (high-Z).
    pub const TRISTATE: u8 = 0;
    /// TX LED.
    pub const TXLED: u8 = 1;
    /// RX LED.
    pub const RXLED: u8 = 2;
    /// TX/RX LED.
    pub const TXRXLED: u8 = 3;
    /// Power Enable.
    pub const PWREN: u8 = 4;
    /// Sleep.
    pub const SLEEP: u8 = 5;
    /// Drive low.
    pub const DRIVE_0: u8 = 6;
    /// Drive high.
    pub const DRIVE_1: u8 = 7;
    /// IO mode.
    pub const IOMODE: u8 = 8;
    /// TX Data Enable.
    pub const TXDEN: u8 = 9;
    /// 30 MHz clock output.
    pub const CLK30: u8 = 10;
    /// 15 MHz clock output.
    pub const CLK15: u8 = 11;
    /// 7.5 MHz clock output.
    pub const CLK7_5: u8 = 12;
}

/// CBUS pin functions for FT230X.
pub mod cbusx {
    /// Tristate (high-Z).
    pub const TRISTATE: u8 = 0;
    /// TX LED.
    pub const TXLED: u8 = 1;
    /// RX LED.
    pub const RXLED: u8 = 2;
    /// TX/RX LED.
    pub const TXRXLED: u8 = 3;
    /// Power Enable.
    pub const PWREN: u8 = 4;
    /// Sleep.
    pub const SLEEP: u8 = 5;
    /// Drive low.
    pub const DRIVE_0: u8 = 6;
    /// Drive high.
    pub const DRIVE_1: u8 = 7;
    /// IO mode.
    pub const IOMODE: u8 = 8;
    /// TX Data Enable.
    pub const TXDEN: u8 = 9;
    /// 24 MHz clock output.
    pub const CLK24: u8 = 10;
    /// 12 MHz clock output.
    pub const CLK12: u8 = 11;
    /// 6 MHz clock output.
    pub const CLK6: u8 = 12;
    /// Battery detect.
    pub const BAT_DETECT: u8 = 13;
    /// Battery detect (negative).
    pub const BAT_DETECT_NEG: u8 = 14;
    /// I2C TX empty.
    pub const I2C_TXE: u8 = 15;
    /// I2C RX full.
    pub const I2C_RXF: u8 = 16;
    /// VBUS sense.
    pub const VBUS_SENSE: u8 = 17;
    /// Bitbang write.
    pub const BB_WR: u8 = 18;
    /// Bitbang read.
    pub const BB_RD: u8 = 19;
    /// Timestamp.
    pub const TIME_STAMP: u8 = 20;
    /// Awake signal.
    pub const AWAKE: u8 = 21;
}

// ---- EEPROM channel / interface mode defines ----

/// Channel mode: UART.
pub const CHANNEL_IS_UART: u8 = 0x0;
/// Channel mode: FIFO.
pub const CHANNEL_IS_FIFO: u8 = 0x1;
/// Channel mode: Opto-isolated.
pub const CHANNEL_IS_OPTO: u8 = 0x2;
/// Channel mode: CPU-style.
pub const CHANNEL_IS_CPU: u8 = 0x4;
/// Channel mode: FT1284.
pub const CHANNEL_IS_FT1284: u8 = 0x8;
/// Channel mode: RS485.
pub const CHANNEL_IS_RS485: u8 = 0x10;

// ---- Drive strength ----

/// 4 mA drive strength.
pub const DRIVE_4MA: u8 = 0;
/// 8 mA drive strength.
pub const DRIVE_8MA: u8 = 1;
/// 12 mA drive strength.
pub const DRIVE_12MA: u8 = 2;
/// 16 mA drive strength.
pub const DRIVE_16MA: u8 = 3;
/// Slow slew rate.
pub const SLOW_SLEW: u8 = 4;
/// Schmitt input.
pub const IS_SCHMITT: u8 = 8;

// ---- Signal inversion bitmask ----

/// Invert TXD.
pub const INVERT_TXD: u8 = 0x01;
/// Invert RXD.
pub const INVERT_RXD: u8 = 0x02;
/// Invert RTS.
pub const INVERT_RTS: u8 = 0x04;
/// Invert CTS.
pub const INVERT_CTS: u8 = 0x08;
/// Invert DTR.
pub const INVERT_DTR: u8 = 0x10;
/// Invert DSR.
pub const INVERT_DSR: u8 = 0x20;
/// Invert DCD.
pub const INVERT_DCD: u8 = 0x40;
/// Invert RI.
pub const INVERT_RI: u8 = 0x80;
