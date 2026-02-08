//! High-level MPSSE (Multi-Protocol Synchronous Serial Engine) API.
//!
//! This module provides typed, ergonomic wrappers around the raw MPSSE commands
//! that FTDI chips (FT2232C/D/H, FT4232H, FT232H) support. It includes:
//!
//! - **MPSSE context**: Initialize MPSSE mode, configure clock, control GPIO pins.
//! - **SPI**: Full-duplex, half-duplex, and bit-level SPI transfers with
//!   configurable CPOL/CPHA and chip-select.
//! - **I2C**: Bit-banged I2C master using MPSSE 3-phase clocking.
//!
//! # Usage
//!
//! ```no_run
//! use ftdi::{FtdiDevice, mpsse::MpsseContext};
//!
//! let mut dev = FtdiDevice::open(0x0403, 0x6014)?; // FT232H
//! let mut mpsse = MpsseContext::init(&mut dev, 1_000_000)?; // 1 MHz
//!
//! // Set GPIOL0 (ADBUS4) as output, drive high
//! mpsse.set_gpio_low(&mut dev, 0x10, 0x10)?;
//!
//! // Read GPIO state
//! let pins = mpsse.get_gpio_low(&mut dev)?;
//! # Ok::<(), ftdi::Error>(())
//! ```

pub mod i2c;
pub mod jtag;
pub mod spi;

use crate::constants::mpsse;
use crate::context::FtdiDevice;
use crate::error::{Error, Result};
use crate::types::{BitMode, ChipType};

/// MPSSE context holding pin state and clock configuration.
///
/// Created by [`MpsseContext::init`]. Stores the current GPIO directions and
/// values so that protocol helpers can manipulate individual pins without
/// clobbering others.
#[derive(Debug, Clone)]
pub struct MpsseContext {
    /// Clock frequency in Hz.
    clock_hz: u32,
    /// Whether the chip is H-type (supports 60 MHz base clock).
    is_h_type: bool,
    /// Low-byte GPIO output values (ADBUS0-7).
    gpio_low_value: u8,
    /// Low-byte GPIO directions (1 = output, 0 = input).
    gpio_low_dir: u8,
    /// High-byte GPIO output values (ACBUS0-7).
    gpio_high_value: u8,
    /// High-byte GPIO directions.
    gpio_high_dir: u8,
}

impl MpsseContext {
    /// Initialize MPSSE mode on the device and configure the clock frequency.
    ///
    /// This performs the standard MPSSE init sequence:
    /// 1. Reset the device mode
    /// 2. Flush buffers
    /// 3. Enable MPSSE mode
    /// 4. Configure the clock frequency
    /// 5. Disable adaptive clocking and loopback
    ///
    /// The `clock_hz` parameter specifies the desired clock frequency.
    /// For H-type chips, frequencies up to 30 MHz are supported (60 MHz base
    /// with div-by-5 disabled). For older chips, up to 6 MHz (12 MHz base).
    ///
    /// Returns an `MpsseContext` that tracks pin state.
    pub fn init(dev: &mut FtdiDevice, clock_hz: u32) -> Result<Self> {
        let chip = dev.chip_type();
        let is_h_type = chip.is_h_type();

        // Verify the chip supports MPSSE
        match chip {
            ChipType::Ft2232C | ChipType::Ft2232H | ChipType::Ft4232H | ChipType::Ft232H => {}
            _ => return Err(Error::UnsupportedChip(chip)),
        }

        // Reset to a known state
        dev.set_bitmode(0, BitMode::Reset)?;
        dev.flush_all()?;

        // Enable MPSSE mode
        dev.set_bitmode(0, BitMode::Mpsse)?;

        // Short delay for the MPSSE engine to start
        std::thread::sleep(std::time::Duration::from_millis(50));

        // Drain any garbage bytes from the MPSSE engine
        let _ = dev.flush_rx();

        let mut ctx = Self {
            clock_hz: 0,
            is_h_type,
            gpio_low_value: 0x00,
            gpio_low_dir: 0x00,
            gpio_high_value: 0x00,
            gpio_high_dir: 0x00,
        };

        // Build init command sequence
        let mut cmd = Vec::with_capacity(16);

        // Disable loopback
        cmd.push(mpsse::LOOPBACK_END);

        // H-type specific setup
        if is_h_type {
            // Disable adaptive clocking
            cmd.push(mpsse::DIS_ADAPTIVE);
        }

        dev.write_all(&cmd)?;

        // Configure the clock
        ctx.set_clock(dev, clock_hz)?;

        Ok(ctx)
    }

    /// Get the current clock frequency in Hz.
    pub fn clock_hz(&self) -> u32 {
        self.clock_hz
    }

    /// Set the MPSSE clock frequency.
    ///
    /// For H-type chips, frequencies up to 30 MHz are supported.
    /// For older chips, up to 6 MHz.
    pub fn set_clock(&mut self, dev: &mut FtdiDevice, clock_hz: u32) -> Result<()> {
        if clock_hz == 0 {
            return Err(Error::InvalidArgument("clock frequency must be > 0"));
        }

        let max_freq = if self.is_h_type {
            30_000_000
        } else {
            6_000_000
        };
        if clock_hz > max_freq {
            return Err(Error::InvalidArgument(
                "clock frequency exceeds maximum for this chip",
            ));
        }

        let mut cmd = Vec::with_capacity(8);

        if self.is_h_type {
            // H-type: base clock is 60 MHz, with optional div-by-5
            // If requested freq > 6 MHz, use 60 MHz base (disable div-by-5)
            // Otherwise, use 12 MHz base (enable div-by-5) for better resolution
            if clock_hz > 6_000_000 {
                cmd.push(mpsse::DIS_DIV_5);
                // 60 MHz base: freq = 60_000_000 / ((1 + divisor) * 2)
                let divisor = (60_000_000u32 / clock_hz.saturating_mul(2)).saturating_sub(1);
                let divisor = divisor.min(0xFFFF) as u16;
                cmd.push(mpsse::TCK_DIVISOR);
                cmd.push(divisor as u8);
                cmd.push((divisor >> 8) as u8);
                self.clock_hz = 60_000_000 / ((1 + divisor as u32) * 2);
            } else {
                cmd.push(mpsse::EN_DIV_5);
                // 12 MHz base: freq = 12_000_000 / ((1 + divisor) * 2)
                let divisor = (12_000_000u32 / clock_hz.saturating_mul(2)).saturating_sub(1);
                let divisor = divisor.min(0xFFFF) as u16;
                cmd.push(mpsse::TCK_DIVISOR);
                cmd.push(divisor as u8);
                cmd.push((divisor >> 8) as u8);
                self.clock_hz = 12_000_000 / ((1 + divisor as u32) * 2);
            }
        } else {
            // Non-H: base clock is 12 MHz
            // freq = 12_000_000 / ((1 + divisor) * 2)
            let divisor = (12_000_000u32 / clock_hz.saturating_mul(2)).saturating_sub(1);
            let divisor = divisor.min(0xFFFF) as u16;
            cmd.push(mpsse::TCK_DIVISOR);
            cmd.push(divisor as u8);
            cmd.push((divisor >> 8) as u8);
            self.clock_hz = 12_000_000 / ((1 + divisor as u32) * 2);
        }

        dev.write_all(&cmd)
    }

    /// Enable 3-phase data clocking (H-type chips only).
    ///
    /// Required for I2C communication. When enabled, data is valid on both
    /// clock edges, allowing it to be read during the high phase of the clock.
    pub fn enable_3phase_clocking(&self, dev: &mut FtdiDevice) -> Result<()> {
        if !self.is_h_type {
            return Err(Error::InvalidArgument(
                "3-phase clocking only supported on H-type chips",
            ));
        }
        dev.write_all(&[mpsse::EN_3_PHASE])
    }

    /// Disable 3-phase data clocking.
    pub fn disable_3phase_clocking(&self, dev: &mut FtdiDevice) -> Result<()> {
        if !self.is_h_type {
            return Err(Error::InvalidArgument(
                "3-phase clocking only supported on H-type chips",
            ));
        }
        dev.write_all(&[mpsse::DIS_3_PHASE])
    }

    /// Enable loopback mode (connect TDI to TDO internally).
    ///
    /// Useful for testing MPSSE communication without external hardware.
    pub fn enable_loopback(&self, dev: &mut FtdiDevice) -> Result<()> {
        dev.write_all(&[mpsse::LOOPBACK_START])
    }

    /// Disable loopback mode.
    pub fn disable_loopback(&self, dev: &mut FtdiDevice) -> Result<()> {
        dev.write_all(&[mpsse::LOOPBACK_END])
    }

    // ---- GPIO Control ----

    /// Set the low-byte GPIO pins (ADBUS0-7).
    ///
    /// `value` sets the output pin states. `direction` sets which pins are
    /// outputs (1) or inputs (0).
    ///
    /// Note: ADBUS0 (SK/SCK), ADBUS1 (DO/MOSI), and ADBUS2 (DI/MISO) are
    /// typically used by the MPSSE protocol. ADBUS3-7 are available as GPIO.
    pub fn set_gpio_low(&mut self, dev: &mut FtdiDevice, value: u8, direction: u8) -> Result<()> {
        self.gpio_low_value = value;
        self.gpio_low_dir = direction;
        dev.write_all(&[mpsse::SET_BITS_LOW, value, direction])
    }

    /// Read the low-byte GPIO pin states (ADBUS0-7).
    pub fn get_gpio_low(&self, dev: &mut FtdiDevice) -> Result<u8> {
        dev.write_all(&[mpsse::GET_BITS_LOW, mpsse::SEND_IMMEDIATE])?;
        let mut buf = [0u8; 1];
        let n = dev.read_data(&mut buf)?;
        if n == 0 {
            return Err(Error::DeviceUnavailable);
        }
        Ok(buf[0])
    }

    /// Set the high-byte GPIO pins (ACBUS0-7).
    ///
    /// Only available on chips with a high byte (FT2232H, FT4232H, FT232H).
    pub fn set_gpio_high(&mut self, dev: &mut FtdiDevice, value: u8, direction: u8) -> Result<()> {
        self.gpio_high_value = value;
        self.gpio_high_dir = direction;
        dev.write_all(&[mpsse::SET_BITS_HIGH, value, direction])
    }

    /// Read the high-byte GPIO pin states (ACBUS0-7).
    pub fn get_gpio_high(&self, dev: &mut FtdiDevice) -> Result<u8> {
        dev.write_all(&[mpsse::GET_BITS_HIGH, mpsse::SEND_IMMEDIATE])?;
        let mut buf = [0u8; 1];
        let n = dev.read_data(&mut buf)?;
        if n == 0 {
            return Err(Error::DeviceUnavailable);
        }
        Ok(buf[0])
    }

    /// Get the current low-byte GPIO direction mask.
    pub fn gpio_low_dir(&self) -> u8 {
        self.gpio_low_dir
    }

    /// Get the current low-byte GPIO output values.
    pub fn gpio_low_value(&self) -> u8 {
        self.gpio_low_value
    }

    /// Get the current high-byte GPIO direction mask.
    pub fn gpio_high_dir(&self) -> u8 {
        self.gpio_high_dir
    }

    /// Get the current high-byte GPIO output values.
    pub fn gpio_high_value(&self) -> u8 {
        self.gpio_high_value
    }

    /// Whether this is an H-type chip.
    pub fn is_h_type(&self) -> bool {
        self.is_h_type
    }

    /// Update the tracked low-byte GPIO state without sending any USB commands.
    ///
    /// Used by protocol helpers (e.g., I2C) that emit raw `SET_BITS_LOW`
    /// commands directly and need to keep the context state in sync.
    pub(crate) fn update_gpio_low_state(&mut self, value: u8, direction: u8) {
        self.gpio_low_value = value;
        self.gpio_low_dir = direction;
    }

    // ---- Raw MPSSE command helpers ----

    /// The MPSSE bad-command response marker byte.
    ///
    /// When the MPSSE engine encounters an unknown opcode, it echoes back
    /// `0xFA` followed by the rejected opcode byte.
    pub const BAD_COMMAND: u8 = 0xFA;

    /// Check a response buffer for a bad-command marker (0xFA).
    ///
    /// If found, returns `Err(Error::MpsseBadCommand(opcode))` with the
    /// rejected opcode. Otherwise returns `Ok(())`.
    ///
    /// **Important:** Only use this on responses where no read data was
    /// requested (e.g., after write-only MPSSE commands). If the response
    /// contains payload data (SPI reads, JTAG TDO, GPIO reads), a
    /// legitimate `0xFA` byte in the data will be misidentified as a
    /// bad-command marker.
    ///
    /// For most users, [`sync_mpsse`](Self::sync_mpsse) is the recommended
    /// way to detect and recover from MPSSE errors.
    pub fn check_bad_command(response: &[u8]) -> Result<()> {
        for i in 0..response.len() {
            if response[i] == Self::BAD_COMMAND && i + 1 < response.len() {
                return Err(Error::MpsseBadCommand(response[i + 1]));
            }
        }
        Ok(())
    }

    /// Synchronize with the MPSSE engine by sending a bogus opcode and
    /// reading until the expected bad-command echo is received.
    ///
    /// This is useful after initialization or error recovery to ensure
    /// the command/response streams are aligned. The bogus opcode `0xAB`
    /// is used (same as libftdi's `ftdi_usb_purge_buffers` test).
    ///
    /// Returns `Ok(())` if synchronization succeeded, or an error if the
    /// expected response was not received.
    pub fn sync_mpsse(&self, dev: &mut FtdiDevice) -> Result<()> {
        const BOGUS_CMD: u8 = 0xAB;

        // Send a known-bad command + flush
        dev.write_all(&[BOGUS_CMD, mpsse::SEND_IMMEDIATE])?;

        // Read response, looking for 0xFA 0xAB
        let mut buf = [0u8; 64];
        let mut attempts = 0;
        while attempts < 10 {
            let n = dev.read_data(&mut buf)?;
            if n >= 2 {
                for i in 0..n - 1 {
                    if buf[i] == Self::BAD_COMMAND && buf[i + 1] == BOGUS_CMD {
                        return Ok(());
                    }
                }
            }
            attempts += 1;
            // Give the device time to process and respond
            std::thread::sleep(std::time::Duration::from_millis(10));
        }

        Err(Error::InvalidArgument(
            "MPSSE synchronization failed: no bad-command echo received",
        ))
    }

    /// Write raw MPSSE commands and then read a response.
    ///
    /// Appends `SEND_IMMEDIATE` after the commands, writes them, then reads
    /// `read_len` bytes of response. This is a convenience for command-response
    /// patterns.
    ///
    /// **Note:** This method does **not** automatically check for bad-command
    /// responses (0xFA), since the response buffer may contain arbitrary
    /// payload data where `0xFA` is a valid value. If you need bad-command
    /// detection, use [`check_bad_command`](Self::check_bad_command) on
    /// responses that do not contain read payload data, or use
    /// [`sync_mpsse`](Self::sync_mpsse) for error recovery.
    pub fn command_response(
        &self,
        dev: &mut FtdiDevice,
        cmd: &[u8],
        read_len: usize,
    ) -> Result<Vec<u8>> {
        let mut full_cmd = Vec::with_capacity(cmd.len() + 1);
        full_cmd.extend_from_slice(cmd);
        full_cmd.push(mpsse::SEND_IMMEDIATE);
        dev.write_all(&full_cmd)?;

        let mut buf = vec![0u8; read_len];
        let mut offset = 0;
        while offset < read_len {
            let n = dev.read_data(&mut buf[offset..])?;
            if n == 0 {
                break;
            }
            offset += n;
        }
        buf.truncate(offset);
        Ok(buf)
    }

    /// Write raw MPSSE command bytes to the device.
    pub fn write_commands(&self, dev: &mut FtdiDevice, cmd: &[u8]) -> Result<()> {
        dev.write_all(cmd)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn clock_divisor_calculations() {
        // Test the divisor math matches expected values
        // 12 MHz base: freq = 12_000_000 / ((1 + div) * 2)
        // For 1 MHz: div = 12_000_000 / (1_000_000 * 2) - 1 = 5
        let div = (12_000_000u32 / (1_000_000 * 2)).saturating_sub(1);
        assert_eq!(div, 5);
        let actual = 12_000_000 / ((1 + div) * 2);
        assert_eq!(actual, 1_000_000);

        // For 100 kHz: div = 12_000_000 / (100_000 * 2) - 1 = 59
        let div = (12_000_000u32 / (100_000 * 2)).saturating_sub(1);
        assert_eq!(div, 59);
        let actual = 12_000_000 / ((1 + div) * 2);
        assert_eq!(actual, 100_000);

        // 60 MHz base for H-type: freq = 60_000_000 / ((1 + div) * 2)
        // For 10 MHz: div = 60_000_000 / (10_000_000 * 2) - 1 = 2
        let div = (60_000_000u32 / (10_000_000 * 2)).saturating_sub(1);
        assert_eq!(div, 2);
        let actual = 60_000_000 / ((1 + div) * 2);
        assert_eq!(actual, 10_000_000);
    }

    #[test]
    fn clock_divisor_30mhz() {
        // 60 MHz base, div=0: freq = 60_000_000 / ((1 + 0) * 2) = 30 MHz
        let div = (60_000_000u32 / (30_000_000 * 2)).saturating_sub(1);
        assert_eq!(div, 0);
        let actual = 60_000_000 / ((1 + div) * 2);
        assert_eq!(actual, 30_000_000);
    }

    #[test]
    fn clock_divisor_6mhz_boundary() {
        // At exactly 6 MHz on H-type, should use 12 MHz base (EN_DIV_5)
        // div = 12_000_000 / (6_000_000 * 2) - 1 = 0
        let div = (12_000_000u32 / (6_000_000 * 2)).saturating_sub(1);
        assert_eq!(div, 0);
        let actual = 12_000_000 / ((1 + div) * 2);
        assert_eq!(actual, 6_000_000);
    }

    #[test]
    fn clock_divisor_400khz_i2c() {
        // 400 kHz I2C: 12 MHz base
        let div = (12_000_000u32 / (400_000 * 2)).saturating_sub(1);
        assert_eq!(div, 14);
        let actual = 12_000_000 / ((1 + div) * 2);
        assert_eq!(actual, 400_000);
    }

    #[test]
    fn mpsse_context_default_state() {
        let ctx = MpsseContext {
            clock_hz: 0,
            is_h_type: true,
            gpio_low_value: 0,
            gpio_low_dir: 0,
            gpio_high_value: 0,
            gpio_high_dir: 0,
        };
        assert_eq!(ctx.gpio_low_value(), 0);
        assert_eq!(ctx.gpio_low_dir(), 0);
        assert_eq!(ctx.gpio_high_value(), 0);
        assert_eq!(ctx.gpio_high_dir(), 0);
        assert!(ctx.is_h_type());
        assert_eq!(ctx.clock_hz(), 0);
    }

    #[test]
    fn update_gpio_low_state_tracks_values() {
        let mut ctx = MpsseContext {
            clock_hz: 1_000_000,
            is_h_type: true,
            gpio_low_value: 0,
            gpio_low_dir: 0,
            gpio_high_value: 0,
            gpio_high_dir: 0,
        };
        ctx.update_gpio_low_state(0xAB, 0xCD);
        assert_eq!(ctx.gpio_low_value(), 0xAB);
        assert_eq!(ctx.gpio_low_dir(), 0xCD);
    }

    #[test]
    fn mpsse_command_constants() {
        // Verify key MPSSE command byte values match AN_108 specification
        assert_eq!(mpsse::SET_BITS_LOW, 0x80);
        assert_eq!(mpsse::GET_BITS_LOW, 0x81);
        assert_eq!(mpsse::SET_BITS_HIGH, 0x82);
        assert_eq!(mpsse::GET_BITS_HIGH, 0x83);
        assert_eq!(mpsse::LOOPBACK_START, 0x84);
        assert_eq!(mpsse::LOOPBACK_END, 0x85);
        assert_eq!(mpsse::TCK_DIVISOR, 0x86);
        assert_eq!(mpsse::SEND_IMMEDIATE, 0x87);
        assert_eq!(mpsse::DIS_DIV_5, 0x8A);
        assert_eq!(mpsse::EN_DIV_5, 0x8B);
        assert_eq!(mpsse::EN_3_PHASE, 0x8C);
        assert_eq!(mpsse::DIS_3_PHASE, 0x8D);
        assert_eq!(mpsse::DIS_ADAPTIVE, 0x97);
    }

    #[test]
    fn mpsse_shifting_flags() {
        assert_eq!(mpsse::WRITE_NEG, 0x01);
        assert_eq!(mpsse::BITMODE, 0x02);
        assert_eq!(mpsse::READ_NEG, 0x04);
        assert_eq!(mpsse::LSB, 0x08);
        assert_eq!(mpsse::DO_WRITE, 0x10);
        assert_eq!(mpsse::DO_READ, 0x20);
        assert_eq!(mpsse::WRITE_TMS, 0x40);
    }

    #[test]
    fn div_value_helper() {
        // 1 MHz: 6_000_000 / 1_000_000 - 1 = 5
        assert_eq!(mpsse::div_value(1_000_000), 5);
        // 6 MHz: 6_000_000 / 6_000_000 - 1 = 0
        assert_eq!(mpsse::div_value(6_000_000), 0);
        // > 6 MHz: returns 0
        assert_eq!(mpsse::div_value(10_000_000), 0);
        // 0 Hz: returns max
        assert_eq!(mpsse::div_value(0), 0xFFFF);
    }

    // ---- Bad-command detection tests ----

    #[test]
    fn check_bad_command_empty() {
        assert!(MpsseContext::check_bad_command(&[]).is_ok());
    }

    #[test]
    fn check_bad_command_normal_data() {
        assert!(MpsseContext::check_bad_command(&[0x00, 0x01, 0xFF]).is_ok());
    }

    #[test]
    fn check_bad_command_detected() {
        let response = [0xFA, 0xAB]; // bad command echo for opcode 0xAB
        let err = MpsseContext::check_bad_command(&response).unwrap_err();
        match err {
            crate::error::Error::MpsseBadCommand(opcode) => assert_eq!(opcode, 0xAB),
            _ => panic!("expected MpsseBadCommand error, got {:?}", err),
        }
    }

    #[test]
    fn check_bad_command_in_middle_of_data() {
        let response = [0x01, 0x02, 0xFA, 0x99, 0x03];
        let err = MpsseContext::check_bad_command(&response).unwrap_err();
        match err {
            crate::error::Error::MpsseBadCommand(opcode) => assert_eq!(opcode, 0x99),
            _ => panic!("expected MpsseBadCommand error"),
        }
    }

    #[test]
    fn check_bad_command_fa_at_end_no_match() {
        // 0xFA at end with no following byte — not a bad-command marker
        let response = [0x01, 0x02, 0xFA];
        assert!(MpsseContext::check_bad_command(&response).is_ok());
    }

    #[test]
    fn bad_command_constant() {
        assert_eq!(MpsseContext::BAD_COMMAND, 0xFA);
    }
}
