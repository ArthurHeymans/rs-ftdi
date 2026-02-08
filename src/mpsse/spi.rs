//! SPI protocol helpers using MPSSE.
//!
//! Provides high-level SPI master operations using the FTDI MPSSE engine.
//! Supports configurable clock polarity (CPOL), clock phase (CPHA), bit order,
//! and chip-select pin management.
//!
//! # Pin Mapping
//!
//! | FTDI Pin | SPI Signal | ADBUS Bit |
//! |----------|-----------|-----------|
//! | SK       | SCLK      | 0         |
//! | DO       | MOSI      | 1         |
//! | DI       | MISO      | 2         |
//! | CS#      | CS (user) | 3-7       |
//!
//! # Example
//!
//! ```no_run
//! use ftdi::{FtdiDevice, mpsse::{MpsseContext, spi::{SpiDevice, SpiMode}}};
//!
//! let mut dev = FtdiDevice::open(0x0403, 0x6014)?;
//! let mut mpsse = MpsseContext::init(&mut dev, 1_000_000)?;
//! let spi = SpiDevice::new(&mut mpsse, &mut dev, SpiMode::Mode0)?;
//!
//! // Write 3 bytes, read 3 bytes (full duplex)
//! let response = spi.transfer(&mut mpsse, &mut dev, &[0x9F, 0x00, 0x00])?;
//!
//! // Write-only (CS automatically asserted/deasserted)
//! spi.write(&mut mpsse, &mut dev, &[0x06])?;
//!
//! // Read-only
//! let data = spi.read(&mut mpsse, &mut dev, 4)?;
//! # Ok::<(), ftdi::Error>(())
//! ```

use crate::constants::mpsse;
use crate::context::FtdiDevice;
use crate::error::{Error, Result};

use super::MpsseContext;

/// Maximum bytes per single MPSSE transfer command (2-byte length field, encoding len-1).
const MAX_MPSSE_TRANSFER: usize = 65536;

/// Read exactly `len` bytes from the MPSSE, returning an error on short reads.
fn read_exact(dev: &mut FtdiDevice, len: usize) -> Result<Vec<u8>> {
    let mut buf = vec![0u8; len];
    let mut offset = 0;
    while offset < len {
        let n = dev.read_data(&mut buf[offset..])?;
        if n == 0 {
            return Err(Error::InvalidArgument(
                "SPI read returned fewer bytes than expected",
            ));
        }
        offset += n;
    }
    Ok(buf)
}

/// Encode a chunk length into the 2-byte MPSSE length field (len-1, little-endian).
#[inline]
fn encode_len(len: usize) -> (u8, u8) {
    let v = (len - 1) as u16;
    (v as u8, (v >> 8) as u8)
}

/// SPI clock polarity and phase mode.
///
/// Standard Motorola SPI modes:
///
/// | Mode | CPOL | CPHA | Description |
/// |------|------|------|-------------|
/// | 0    | 0    | 0    | Clock idle low, sample on rising edge |
/// | 1    | 0    | 1    | Clock idle low, sample on falling edge |
/// | 2    | 1    | 0    | Clock idle high, sample on falling edge |
/// | 3    | 1    | 1    | Clock idle high, sample on rising edge |
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum SpiMode {
    /// CPOL=0, CPHA=0.
    Mode0,
    /// CPOL=0, CPHA=1.
    Mode1,
    /// CPOL=1, CPHA=0.
    Mode2,
    /// CPOL=1, CPHA=1.
    Mode3,
}

impl SpiMode {
    /// Clock polarity: true = idle high.
    pub fn cpol(self) -> bool {
        matches!(self, Self::Mode2 | Self::Mode3)
    }

    /// Clock phase: true = sample on second edge.
    pub fn cpha(self) -> bool {
        matches!(self, Self::Mode1 | Self::Mode3)
    }
}

/// Configuration for an SPI device connected to the MPSSE.
#[derive(Debug, Clone)]
pub struct SpiDevice {
    /// The SPI mode (clock polarity and phase).
    mode: SpiMode,
    /// Whether to use LSB-first bit order (default: MSB first).
    lsb_first: bool,
    /// CS pin bit mask in the low GPIO byte (e.g., 0x08 for ADBUS3).
    cs_pin: u8,
    /// Whether CS is active low (default: true).
    cs_active_low: bool,
    /// MPSSE opcode for write (depends on mode).
    write_cmd: u8,
    /// MPSSE opcode for read (depends on mode).
    read_cmd: u8,
    /// MPSSE opcode for simultaneous read+write (depends on mode).
    rw_cmd: u8,
    /// Initial low-byte direction mask (SK=out, DO=out, DI=in, CS=out).
    dir_mask: u8,
    /// Initial low-byte value (CS deasserted, clock at idle level).
    idle_value: u8,
}

impl SpiDevice {
    /// Create a new SPI device configuration with default CS on ADBUS3.
    ///
    /// Initializes the MPSSE pins for SPI:
    /// - ADBUS0 (SK) = SCLK output
    /// - ADBUS1 (DO) = MOSI output
    /// - ADBUS2 (DI) = MISO input
    /// - ADBUS3 = CS# output (active low, deasserted on init)
    pub fn new(ctx: &mut MpsseContext, dev: &mut FtdiDevice, mode: SpiMode) -> Result<Self> {
        Self::with_cs_pin(ctx, dev, mode, 0x08, true, false)
    }

    /// Create an SPI device with a custom CS pin and options.
    ///
    /// `cs_pin` is the bit mask for the CS pin in the low GPIO byte (e.g.,
    /// 0x08 for ADBUS3, 0x10 for ADBUS4). Set to 0 to manage CS manually.
    ///
    /// `cs_active_low` controls the CS polarity (true = CS is active when low).
    ///
    /// `lsb_first` controls the bit order (true = LSB first, false = MSB first).
    pub fn with_cs_pin(
        ctx: &mut MpsseContext,
        dev: &mut FtdiDevice,
        mode: SpiMode,
        cs_pin: u8,
        cs_active_low: bool,
        lsb_first: bool,
    ) -> Result<Self> {
        // Build MPSSE opcodes based on mode and bit order
        let lsb = if lsb_first { mpsse::LSB } else { 0 };

        // For SPI we use byte-level commands (not BITMODE)
        // MPSSE edge names are physical: WRITE_NEG = shift out on falling SK,
        // READ_NEG = sample in on falling SK. The idle clock level (CPOL) is
        // handled separately via set_gpio_low. Per FTDI AN_108:
        //   Mode 0 (CPOL=0, CPHA=0): data changes on falling, sampled on rising
        //   Mode 1 (CPOL=0, CPHA=1): data changes on rising, sampled on falling
        //   Mode 2 (CPOL=1, CPHA=0): data changes on rising, sampled on falling
        //   Mode 3 (CPOL=1, CPHA=1): data changes on falling, sampled on rising
        let (write_cmd, read_cmd, rw_cmd) = match mode {
            SpiMode::Mode0 | SpiMode::Mode3 => {
                // Data out on falling SK (WRITE_NEG), data in on rising SK
                (
                    mpsse::DO_WRITE | mpsse::WRITE_NEG | lsb,
                    mpsse::DO_READ | lsb,
                    mpsse::DO_WRITE | mpsse::DO_READ | mpsse::WRITE_NEG | lsb,
                )
            }
            SpiMode::Mode1 | SpiMode::Mode2 => {
                // Data out on rising SK, data in on falling SK (READ_NEG)
                (
                    mpsse::DO_WRITE | lsb,
                    mpsse::DO_READ | mpsse::READ_NEG | lsb,
                    mpsse::DO_WRITE | mpsse::DO_READ | mpsse::READ_NEG | lsb,
                )
            }
        };

        // Direction: SK(0)=out, DO(1)=out, DI(2)=in, CS=out
        let dir_mask = 0x03 | cs_pin; // bits 0,1 = output, plus CS pin

        // Idle value: clock at idle level, CS deasserted
        let cs_idle = if cs_active_low { cs_pin } else { 0 }; // deasserted state
        let clk_idle = if mode.cpol() { 0x01 } else { 0x00 }; // SK at idle level
        let idle_value = clk_idle | cs_idle;

        let spi = Self {
            mode,
            lsb_first,
            cs_pin,
            cs_active_low,
            write_cmd,
            read_cmd,
            rw_cmd,
            dir_mask,
            idle_value,
        };

        // Set initial pin state
        ctx.set_gpio_low(dev, idle_value, dir_mask)?;

        Ok(spi)
    }

    /// Assert the chip-select line (make it active).
    pub fn cs_assert(&self, ctx: &mut MpsseContext, dev: &mut FtdiDevice) -> Result<()> {
        if self.cs_pin == 0 {
            return Ok(());
        }
        let value = if self.cs_active_low {
            self.idle_value & !self.cs_pin // drive CS low
        } else {
            self.idle_value | self.cs_pin // drive CS high
        };
        ctx.set_gpio_low(dev, value, self.dir_mask)
    }

    /// Deassert the chip-select line (make it inactive).
    pub fn cs_deassert(&self, ctx: &mut MpsseContext, dev: &mut FtdiDevice) -> Result<()> {
        if self.cs_pin == 0 {
            return Ok(());
        }
        ctx.set_gpio_low(dev, self.idle_value, self.dir_mask)
    }

    /// Full-duplex SPI transfer: simultaneously write `tx` and read the same
    /// number of bytes.
    ///
    /// CS is automatically asserted before and deasserted after the transfer.
    /// Large transfers (>65536 bytes) are automatically split into multiple
    /// MPSSE commands within the same CS assertion.
    /// Returns the received bytes.
    pub fn transfer(
        &self,
        _ctx: &mut MpsseContext,
        dev: &mut FtdiDevice,
        tx: &[u8],
    ) -> Result<Vec<u8>> {
        if tx.is_empty() {
            return Ok(Vec::new());
        }

        let total = tx.len();
        let mut cmd = Vec::with_capacity(10 + total);

        // Assert CS
        self.append_cs_assert(&mut cmd);

        // Emit one or more R/W commands for chunks up to MAX_MPSSE_TRANSFER
        let mut offset = 0;
        while offset < total {
            let chunk_len = (total - offset).min(MAX_MPSSE_TRANSFER);
            let (lo, hi) = encode_len(chunk_len);
            cmd.push(self.rw_cmd);
            cmd.push(lo);
            cmd.push(hi);
            cmd.extend_from_slice(&tx[offset..offset + chunk_len]);
            offset += chunk_len;
        }

        // Deassert CS + flush
        self.append_cs_deassert(&mut cmd);
        cmd.push(mpsse::SEND_IMMEDIATE);

        dev.write_all(&cmd)?;

        read_exact(dev, total)
    }

    /// Write-only SPI transfer.
    ///
    /// CS is automatically asserted before and deasserted after the transfer.
    /// Large transfers (>65536 bytes) are automatically chunked.
    pub fn write(&self, _ctx: &mut MpsseContext, dev: &mut FtdiDevice, tx: &[u8]) -> Result<()> {
        if tx.is_empty() {
            return Ok(());
        }

        let total = tx.len();
        let mut cmd = Vec::with_capacity(10 + total);

        self.append_cs_assert(&mut cmd);

        let mut offset = 0;
        while offset < total {
            let chunk_len = (total - offset).min(MAX_MPSSE_TRANSFER);
            let (lo, hi) = encode_len(chunk_len);
            cmd.push(self.write_cmd);
            cmd.push(lo);
            cmd.push(hi);
            cmd.extend_from_slice(&tx[offset..offset + chunk_len]);
            offset += chunk_len;
        }

        self.append_cs_deassert(&mut cmd);

        dev.write_all(&cmd)
    }

    /// Read-only SPI transfer (writes zeros while reading).
    ///
    /// CS is automatically asserted before and deasserted after the transfer.
    /// Large reads (>65536 bytes) are automatically chunked.
    /// Returns the received bytes.
    pub fn read(
        &self,
        _ctx: &mut MpsseContext,
        dev: &mut FtdiDevice,
        len: usize,
    ) -> Result<Vec<u8>> {
        if len == 0 {
            return Ok(Vec::new());
        }

        let mut cmd = Vec::with_capacity(16);

        self.append_cs_assert(&mut cmd);

        let mut remaining = len;
        while remaining > 0 {
            let chunk_len = remaining.min(MAX_MPSSE_TRANSFER);
            let (lo, hi) = encode_len(chunk_len);
            cmd.push(self.read_cmd);
            cmd.push(lo);
            cmd.push(hi);
            remaining -= chunk_len;
        }

        self.append_cs_deassert(&mut cmd);
        cmd.push(mpsse::SEND_IMMEDIATE);

        dev.write_all(&cmd)?;

        read_exact(dev, len)
    }

    /// Perform a write-then-read SPI transaction with a single CS assertion.
    ///
    /// This is common for SPI devices where you send a command and then
    /// read the response (e.g., reading a register).
    /// Large transfers (>65536 bytes in either direction) are automatically chunked.
    pub fn write_read(
        &self,
        _ctx: &mut MpsseContext,
        dev: &mut FtdiDevice,
        tx: &[u8],
        read_len: usize,
    ) -> Result<Vec<u8>> {
        if tx.is_empty() && read_len == 0 {
            return Ok(Vec::new());
        }

        let mut cmd = Vec::with_capacity(16 + tx.len());

        self.append_cs_assert(&mut cmd);

        // Write phase (chunked)
        if !tx.is_empty() {
            let mut offset = 0;
            while offset < tx.len() {
                let chunk_len = (tx.len() - offset).min(MAX_MPSSE_TRANSFER);
                let (lo, hi) = encode_len(chunk_len);
                cmd.push(self.write_cmd);
                cmd.push(lo);
                cmd.push(hi);
                cmd.extend_from_slice(&tx[offset..offset + chunk_len]);
                offset += chunk_len;
            }
        }

        // Read phase (chunked)
        if read_len > 0 {
            let mut remaining = read_len;
            while remaining > 0 {
                let chunk_len = remaining.min(MAX_MPSSE_TRANSFER);
                let (lo, hi) = encode_len(chunk_len);
                cmd.push(self.read_cmd);
                cmd.push(lo);
                cmd.push(hi);
                remaining -= chunk_len;
            }
        }

        self.append_cs_deassert(&mut cmd);

        if read_len > 0 {
            cmd.push(mpsse::SEND_IMMEDIATE);
        }

        dev.write_all(&cmd)?;

        if read_len == 0 {
            return Ok(Vec::new());
        }

        read_exact(dev, read_len)
    }

    /// Get the current SPI mode.
    pub fn mode(&self) -> SpiMode {
        self.mode
    }

    /// Whether this SPI device uses LSB-first bit order.
    pub fn is_lsb_first(&self) -> bool {
        self.lsb_first
    }

    /// Get the CS pin bit mask.
    pub fn cs_pin(&self) -> u8 {
        self.cs_pin
    }

    /// Append a SET_BITS_LOW command to `cmd` that asserts CS.
    ///
    /// This is a zero-cost helper for building MPSSE command buffers without
    /// needing a mutable reference to `MpsseContext` or `FtdiDevice`.
    /// If `cs_pin` is 0 (manual CS), this is a no-op.
    fn append_cs_assert(&self, cmd: &mut Vec<u8>) {
        if self.cs_pin == 0 {
            return;
        }
        let value = if self.cs_active_low {
            self.idle_value & !self.cs_pin // drive CS low
        } else {
            self.idle_value | self.cs_pin // drive CS high
        };
        cmd.extend_from_slice(&[mpsse::SET_BITS_LOW, value, self.dir_mask]);
    }

    /// Append a SET_BITS_LOW command to `cmd` that deasserts CS (returns to idle).
    ///
    /// If `cs_pin` is 0 (manual CS), this is a no-op.
    fn append_cs_deassert(&self, cmd: &mut Vec<u8>) {
        if self.cs_pin == 0 {
            return;
        }
        cmd.extend_from_slice(&[mpsse::SET_BITS_LOW, self.idle_value, self.dir_mask]);
    }
}
