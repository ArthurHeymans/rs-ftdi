//! Core FTDI device handle and operations.
//!
//! [`FtdiDevice`] is the main type in this crate. It represents an opened,
//! configured FTDI USB device and provides methods for serial communication,
//! bitbang/MPSSE mode, flow control, and EEPROM access.

use std::io;
use std::time::Duration;

use nusb::transfer::{ControlIn, ControlOut, ControlType, Recipient};
use nusb::{self, DeviceInfo, MaybeFuture};

use crate::baudrate;
use crate::constants::*;
use crate::eeprom::FtdiEeprom;
use crate::error::{Error, Result};
use crate::types::*;

/// Default read/write timeout.
const DEFAULT_TIMEOUT: Duration = Duration::from_secs(5);

/// Default read/write buffer chunk size.
const DEFAULT_CHUNKSIZE: usize = 4096;

/// An opened FTDI USB device.
///
/// This is the primary handle for communicating with an FTDI chip.
/// It owns the USB device and interface, manages internal read buffers,
/// and provides methods for all supported operations.
///
/// # Opening a device
///
/// ```no_run
/// use ftdi::FtdiDevice;
///
/// let mut dev = FtdiDevice::open(0x0403, 0x6001)?;
/// dev.set_baudrate(115200)?;
/// dev.write_all(b"Hello FTDI!\r\n")?;
/// # Ok::<(), ftdi::Error>(())
/// ```
///
/// # Implements `Read` and `Write`
///
/// `FtdiDevice` implements [`std::io::Read`] and [`std::io::Write`], so you
/// can use it anywhere those traits are expected.
pub struct FtdiDevice {
    #[allow(dead_code)] // Kept to ensure the USB device stays open
    device: nusb::Device,
    interface: nusb::Interface,

    // Chip identification
    chip_type: ChipType,

    // Transfer configuration
    baudrate: u32,
    bitbang_enabled: bool,
    bitbang_mode: BitMode,
    read_timeout: Duration,
    write_timeout: Duration,

    // Internal read buffer (modem status bytes are stripped here)
    readbuffer: Vec<u8>,
    readbuffer_offset: usize,
    readbuffer_remaining: usize,
    readbuffer_chunksize: usize,
    writebuffer_chunksize: usize,

    // USB endpoint configuration
    max_packet_size: usize,
    interface_num: u8,
    usb_index: u16,
    write_ep: u8, // bulk OUT endpoint (host -> device, for writing)
    read_ep: u8,  // bulk IN endpoint (device -> host, for reading)

    // EEPROM
    pub(crate) eeprom: FtdiEeprom,
}

impl std::fmt::Debug for FtdiDevice {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("FtdiDevice")
            .field("chip_type", &self.chip_type)
            .field("baudrate", &self.baudrate)
            .field("interface", &self.interface_num)
            .field("bitbang_enabled", &self.bitbang_enabled)
            .field("max_packet_size", &self.max_packet_size)
            .finish_non_exhaustive()
    }
}

// ---- Construction / Opening ----

impl FtdiDevice {
    /// Open the first FTDI device matching the given vendor and product IDs.
    ///
    /// Uses [`Interface::A`] by default. For multi-interface chips, use
    /// [`open_with_interface`](Self::open_with_interface).
    pub fn open(vendor: u16, product: u16) -> Result<Self> {
        Self::open_with_interface(vendor, product, Interface::Any)
    }

    /// Open the first matching device on a specific interface.
    pub fn open_with_interface(vendor: u16, product: u16, iface: Interface) -> Result<Self> {
        let dev_info = nusb::list_devices()
            .wait()?
            .find(|d| d.vendor_id() == vendor && d.product_id() == product)
            .ok_or(Error::DeviceNotFound)?;

        Self::from_device_info(dev_info, iface)
    }

    /// Open a device from a [`DeviceFilter`](crate::DeviceFilter).
    pub fn open_with_filter(
        filter: &crate::device_info::DeviceFilter,
        iface: Interface,
    ) -> Result<Self> {
        let dev_info = crate::device_info::find_device(filter)?;
        Self::from_device_info(dev_info, iface)
    }

    /// Open a device by USB bus number and device address.
    pub fn open_bus_addr(bus: u8, addr: u8, iface: Interface) -> Result<Self> {
        let dev_info = nusb::list_devices()
            .wait()?
            .find(|d| d.busnum() == bus && d.device_address() == addr)
            .ok_or(Error::DeviceNotFound)?;

        Self::from_device_info(dev_info, iface)
    }

    /// Open a device from an already-discovered [`nusb::DeviceInfo`].
    pub fn from_device_info(dev_info: DeviceInfo, iface: Interface) -> Result<Self> {
        let config = iface.config();

        let device = dev_info.open().wait()?;

        // Detach kernel driver and claim interface
        let interface = device
            .detach_and_claim_interface(config.interface_num)
            .wait()?;

        // Auto-detect chip type from bcdDevice
        let desc = device.device_descriptor();
        let bcd = desc.device_version();
        let has_serial = desc.serial_number_string_index().is_some();

        let chip_type = match bcd {
            0x0400 => ChipType::Bm,
            0x0200 if !has_serial => ChipType::Bm, // Bug in BM: bcdDevice=0x200 when serial==0
            0x0200 => ChipType::Am,
            0x0500 => ChipType::Ft2232C,
            0x0600 => ChipType::Ft232R,
            0x0700 => ChipType::Ft2232H,
            0x0800 => ChipType::Ft4232H,
            0x0900 => ChipType::Ft232H,
            0x1000 => ChipType::Ft230X,
            _ => ChipType::Bm, // Default fallback
        };

        // Determine max packet size from descriptors
        let max_packet_size = determine_max_packet_size(&device, chip_type, config.interface_num);

        let mut ftdi = Self {
            device,
            interface,
            chip_type,
            baudrate: 0,
            bitbang_enabled: false,
            bitbang_mode: BitMode::Reset,
            read_timeout: DEFAULT_TIMEOUT,
            write_timeout: DEFAULT_TIMEOUT,
            readbuffer: vec![0u8; DEFAULT_CHUNKSIZE],
            readbuffer_offset: 0,
            readbuffer_remaining: 0,
            readbuffer_chunksize: DEFAULT_CHUNKSIZE,
            writebuffer_chunksize: DEFAULT_CHUNKSIZE,
            max_packet_size,
            interface_num: config.interface_num,
            usb_index: config.usb_index,
            write_ep: config.write_ep,
            read_ep: config.read_ep,
            eeprom: FtdiEeprom::default(),
        };

        // Reset device
        ftdi.usb_reset()?;

        // Set default baud rate
        ftdi.set_baudrate(9600)?;

        Ok(ftdi)
    }

    /// The detected FTDI chip type.
    pub fn chip_type(&self) -> ChipType {
        self.chip_type
    }

    /// The currently configured baud rate.
    pub fn baudrate(&self) -> u32 {
        self.baudrate
    }

    /// The maximum USB packet size for this device.
    pub fn max_packet_size(&self) -> usize {
        self.max_packet_size
    }
}

// ---- Internal USB helpers ----

impl FtdiDevice {
    /// Open the bulk IN endpoint (device -> host) for reading.
    ///
    /// This is used by the streaming and async modules and should not be called directly.
    pub(crate) fn bulk_in_endpoint(
        &self,
    ) -> Result<nusb::Endpoint<nusb::transfer::Bulk, nusb::transfer::In>> {
        self.interface
            .endpoint::<nusb::transfer::Bulk, nusb::transfer::In>(self.read_ep)
            .map_err(Error::Usb)
    }

    /// Open the bulk OUT endpoint (host -> device) for writing.
    ///
    /// This is used by the async module and should not be called directly.
    pub(crate) fn bulk_out_endpoint(
        &self,
    ) -> Result<nusb::Endpoint<nusb::transfer::Bulk, nusb::transfer::Out>> {
        self.interface
            .endpoint::<nusb::transfer::Bulk, nusb::transfer::Out>(self.write_ep)
            .map_err(Error::Usb)
    }

    /// Get the write buffer chunk size (for internal use by async module).
    pub(crate) fn writebuffer_chunksize(&self) -> usize {
        self.writebuffer_chunksize
    }

    /// Get the read buffer chunk size (for internal use by async module).
    pub(crate) fn readbuffer_chunksize(&self) -> usize {
        self.readbuffer_chunksize
    }

    /// Drain up to `n` bytes from the internal read buffer (for async module).
    pub(crate) fn drain_readbuffer(&mut self, max: usize) -> Vec<u8> {
        let n = self.readbuffer_remaining.min(max);
        if n == 0 {
            return Vec::new();
        }
        let data = self.readbuffer[self.readbuffer_offset..self.readbuffer_offset + n].to_vec();
        self.readbuffer_remaining -= n;
        self.readbuffer_offset += n;
        data
    }

    /// Send a vendor OUT control transfer to the device.
    pub(crate) fn control_out(&self, request: u8, value: u16, index: u16) -> Result<()> {
        self.interface
            .control_out(
                ControlOut {
                    control_type: ControlType::Vendor,
                    recipient: Recipient::Device,
                    request,
                    value,
                    index,
                    data: &[],
                },
                self.write_timeout,
            )
            .wait()?;
        Ok(())
    }

    /// Send a vendor IN control transfer to the device.
    pub(crate) fn control_in(
        &self,
        request: u8,
        value: u16,
        index: u16,
        length: u16,
    ) -> Result<Vec<u8>> {
        let data = self
            .interface
            .control_in(
                ControlIn {
                    control_type: ControlType::Vendor,
                    recipient: Recipient::Device,
                    request,
                    value,
                    index,
                    length,
                },
                self.read_timeout,
            )
            .wait()?;
        Ok(data)
    }
}

// ---- Reset / Flush ----

impl FtdiDevice {
    /// Perform a USB reset on the FTDI device.
    ///
    /// This resets the device to its default state. The internal read buffer
    /// is invalidated.
    pub fn usb_reset(&mut self) -> Result<()> {
        self.control_out(SIO_RESET_REQUEST, SIO_RESET_SIO, self.usb_index)?;
        self.readbuffer_offset = 0;
        self.readbuffer_remaining = 0;
        Ok(())
    }

    /// Flush the receive (RX) buffer.
    ///
    /// Clears data in the chip's RX FIFO (data flowing from the serial
    /// device toward the host) and the internal software read buffer.
    pub fn flush_rx(&mut self) -> Result<()> {
        self.control_out(SIO_RESET_REQUEST, SIO_TCIFLUSH, self.usb_index)?;
        self.readbuffer_offset = 0;
        self.readbuffer_remaining = 0;
        Ok(())
    }

    /// Flush the transmit (TX) buffer.
    ///
    /// Clears data in the chip's TX FIFO (data flowing from the host
    /// toward the serial device).
    pub fn flush_tx(&mut self) -> Result<()> {
        self.control_out(SIO_RESET_REQUEST, SIO_TCOFLUSH, self.usb_index)?;
        Ok(())
    }

    /// Flush both RX and TX buffers.
    pub fn flush_all(&mut self) -> Result<()> {
        self.flush_rx()?;
        self.flush_tx()
    }
}

// ---- Serial Configuration ----

impl FtdiDevice {
    /// Set the baud rate.
    ///
    /// The actual baud rate achieved is determined by the chip's clock
    /// divider and may differ slightly from the requested value. An error
    /// is returned if the achievable rate deviates by more than ~5%.
    ///
    /// When bitbang mode is enabled, the baud rate is internally multiplied
    /// by 4 (the FTDI chip's bitbang clock runs at 4x the serial baud rate).
    pub fn set_baudrate(&mut self, baudrate: u32) -> Result<()> {
        let effective = if self.bitbang_enabled {
            baudrate * 4
        } else {
            baudrate
        };

        let result = baudrate::convert_baudrate(effective, self.chip_type, self.usb_index)
            .ok_or(Error::InvalidArgument("baud rate must be > 0"))?;

        // Check within ~5% tolerance
        let actual = result.actual;
        if (actual as u64) * 2 < effective as u64
            || if actual < effective {
                (actual as u64) * 21 < (effective as u64) * 20
            } else {
                (effective as u64) * 21 < (actual as u64) * 20
            }
        {
            return Err(Error::UnsupportedBaudRate {
                requested: baudrate,
                actual,
            });
        }

        self.control_out(SIO_SET_BAUDRATE_REQUEST, result.value, result.index)?;
        self.baudrate = baudrate;
        Ok(())
    }

    /// Set the serial line properties (data bits, stop bits, parity).
    pub fn set_line_property(
        &self,
        bits: DataBits,
        stop_bits: StopBits,
        parity: Parity,
    ) -> Result<()> {
        self.set_line_property_with_break(bits, stop_bits, parity, BreakType::Off)
    }

    /// Set the serial line properties including break control.
    pub fn set_line_property_with_break(
        &self,
        bits: DataBits,
        stop_bits: StopBits,
        parity: Parity,
        break_type: BreakType,
    ) -> Result<()> {
        let value = bits.wire_value()
            | (parity.wire_value() << 8)
            | (stop_bits.wire_value() << 11)
            | (break_type.wire_value() << 14);

        self.control_out(SIO_SET_DATA_REQUEST, value, self.usb_index)
    }

    /// Set the read timeout for USB transfers.
    pub fn set_read_timeout(&mut self, timeout: Duration) {
        self.read_timeout = timeout;
    }

    /// Set the write timeout for USB transfers.
    pub fn set_write_timeout(&mut self, timeout: Duration) {
        self.write_timeout = timeout;
    }

    /// Get the current read timeout.
    pub fn read_timeout(&self) -> Duration {
        self.read_timeout
    }

    /// Get the current write timeout.
    pub fn write_timeout(&self) -> Duration {
        self.write_timeout
    }
}

// ---- Flow Control / Modem Lines ----

impl FtdiDevice {
    /// Set the flow control mode.
    ///
    /// For XON/XOFF flow control, use [`set_flow_control_xonxoff`](Self::set_flow_control_xonxoff).
    pub fn set_flow_control(&self, flow: FlowControl) -> Result<()> {
        let flow_val = match flow {
            FlowControl::Disabled => SIO_DISABLE_FLOW_CTRL,
            FlowControl::RtsCts => SIO_RTS_CTS_HS,
            FlowControl::DtrDsr => SIO_DTR_DSR_HS,
        };

        self.control_out(SIO_SET_FLOW_CTRL_REQUEST, 0, flow_val | self.usb_index)
    }

    /// Set XON/XOFF software flow control with custom characters.
    pub fn set_flow_control_xonxoff(&self, xon: u8, xoff: u8) -> Result<()> {
        let xonxoff = (xon as u16) | ((xoff as u16) << 8);
        self.control_out(
            SIO_SET_FLOW_CTRL_REQUEST,
            xonxoff,
            SIO_XON_XOFF_HS | self.usb_index,
        )
    }

    /// Set the DTR (Data Terminal Ready) line state.
    pub fn set_dtr(&self, state: bool) -> Result<()> {
        let val = if state {
            SIO_SET_DTR_HIGH
        } else {
            SIO_SET_DTR_LOW
        };
        self.control_out(SIO_SET_MODEM_CTRL_REQUEST, val, self.usb_index)
    }

    /// Set the RTS (Request To Send) line state.
    pub fn set_rts(&self, state: bool) -> Result<()> {
        let val = if state {
            SIO_SET_RTS_HIGH
        } else {
            SIO_SET_RTS_LOW
        };
        self.control_out(SIO_SET_MODEM_CTRL_REQUEST, val, self.usb_index)
    }

    /// Set both DTR and RTS lines in a single USB transfer.
    pub fn set_dtr_rts(&self, dtr: bool, rts: bool) -> Result<()> {
        let mut val = if dtr {
            SIO_SET_DTR_HIGH
        } else {
            SIO_SET_DTR_LOW
        };
        val |= if rts {
            SIO_SET_RTS_HIGH
        } else {
            SIO_SET_RTS_LOW
        };
        self.control_out(SIO_SET_MODEM_CTRL_REQUEST, val, self.usb_index)
    }

    /// Set the special event character.
    ///
    /// When `enable` is true and this character is received, the chip
    /// immediately returns the data up to and including this character.
    pub fn set_event_char(&self, ch: u8, enable: bool) -> Result<()> {
        let val = (ch as u16) | if enable { 1 << 8 } else { 0 };
        self.control_out(SIO_SET_EVENT_CHAR_REQUEST, val, self.usb_index)
    }

    /// Set the error character.
    ///
    /// When enabled, this character is inserted into the data stream
    /// when a parity error is detected.
    pub fn set_error_char(&self, ch: u8, enable: bool) -> Result<()> {
        let val = (ch as u16) | if enable { 1 << 8 } else { 0 };
        self.control_out(SIO_SET_ERROR_CHAR_REQUEST, val, self.usb_index)
    }

    /// Poll the modem status.
    ///
    /// Returns the current state of the modem control lines and line
    /// status register. This bypasses the normal read buffer.
    pub fn poll_modem_status(&self) -> Result<ModemStatus> {
        let data = self.control_in(SIO_POLL_MODEM_STATUS_REQUEST, 0, self.usb_index, 2)?;
        if data.len() < 2 {
            return Err(Error::DeviceUnavailable);
        }
        let raw = (data[0] as u16) | ((data[1] as u16) << 8);
        Ok(ModemStatus::from_raw(raw))
    }
}

// ---- Latency Timer ----

impl FtdiDevice {
    /// Set the latency timer value (1-255 ms).
    ///
    /// The FTDI chip holds data in its internal buffer for this duration
    /// if the buffer is not full, to reduce USB bus load.
    pub fn set_latency_timer(&self, latency_ms: u8) -> Result<()> {
        if latency_ms < 1 {
            return Err(Error::InvalidArgument("latency must be between 1 and 255"));
        }
        self.control_out(
            SIO_SET_LATENCY_TIMER_REQUEST,
            latency_ms as u16,
            self.usb_index,
        )
    }

    /// Get the current latency timer value in milliseconds.
    pub fn latency_timer(&self) -> Result<u8> {
        let data = self.control_in(SIO_GET_LATENCY_TIMER_REQUEST, 0, self.usb_index, 1)?;
        if data.is_empty() {
            return Err(Error::DeviceUnavailable);
        }
        Ok(data[0])
    }
}

// ---- Bitbang / MPSSE ----

impl FtdiDevice {
    /// Enable a bitbang or MPSSE mode.
    ///
    /// The `bitmask` configures which pins are outputs (bit = 1) and which
    /// are inputs (bit = 0). The `mode` selects the operating mode.
    ///
    /// To return to normal serial mode, call [`disable_bitbang`](Self::disable_bitbang)
    /// or use `BitMode::Reset`.
    pub fn set_bitmode(&mut self, bitmask: u8, mode: BitMode) -> Result<()> {
        let val = (bitmask as u16) | ((mode.wire_value() as u16) << 8);
        self.control_out(SIO_SET_BITMODE_REQUEST, val, self.usb_index)?;

        self.bitbang_mode = mode;
        self.bitbang_enabled = mode != BitMode::Reset;
        Ok(())
    }

    /// Disable bitbang mode and return to normal serial/FIFO operation.
    pub fn disable_bitbang(&mut self) -> Result<()> {
        self.set_bitmode(0, BitMode::Reset)
    }

    /// Read the current pin states directly, bypassing the read buffer.
    ///
    /// This is useful in bitbang mode to sample the instantaneous pin values.
    pub fn read_pins(&self) -> Result<u8> {
        let data = self.control_in(SIO_READ_PINS_REQUEST, 0, self.usb_index, 1)?;
        if data.is_empty() {
            return Err(Error::DeviceUnavailable);
        }
        Ok(data[0])
    }
}

// ---- Chunk Size Configuration ----

impl FtdiDevice {
    /// Set the read buffer chunk size.
    ///
    /// This controls how many bytes are requested in each USB bulk read.
    /// The default is 4096. The internal read buffer is reallocated to
    /// match.
    pub fn set_read_chunksize(&mut self, chunksize: usize) {
        self.readbuffer_offset = 0;
        self.readbuffer_remaining = 0;
        self.readbuffer_chunksize = chunksize;
        self.readbuffer.resize(chunksize, 0);
    }

    /// Get the current read buffer chunk size.
    pub fn read_chunksize(&self) -> usize {
        self.readbuffer_chunksize
    }

    /// Set the write buffer chunk size.
    ///
    /// This controls the maximum number of bytes sent in each USB bulk write.
    /// The default is 4096.
    pub fn set_write_chunksize(&mut self, chunksize: usize) {
        self.writebuffer_chunksize = chunksize;
    }

    /// Get the current write buffer chunk size.
    pub fn write_chunksize(&self) -> usize {
        self.writebuffer_chunksize
    }
}

// ---- Data Transfer ----

impl FtdiDevice {
    /// Write data to the FTDI device.
    ///
    /// Data is sent in chunks of [`write_chunksize`](Self::write_chunksize).
    /// Returns the number of bytes written.
    pub fn write_data(&mut self, buf: &[u8]) -> Result<usize> {
        use nusb::transfer::{Bulk, Out};

        let mut offset = 0;
        let mut ep = self
            .interface
            .endpoint::<Bulk, Out>(self.write_ep)
            .map_err(Error::Usb)?;

        while offset < buf.len() {
            let end = (offset + self.writebuffer_chunksize).min(buf.len());
            let chunk = &buf[offset..end];

            let mut transfer_buf = nusb::transfer::Buffer::new(chunk.len());
            transfer_buf.extend_from_slice(chunk);

            let completion = ep.transfer_blocking(transfer_buf, self.write_timeout);
            completion.status.map_err(Error::Transfer)?;
            offset += completion.actual_len;
        }

        Ok(offset)
    }

    /// Read data from the FTDI device.
    ///
    /// Automatically strips the two modem status bytes that the FTDI chip
    /// prepends to every USB packet. Returns the number of payload bytes
    /// read into `buf`.
    ///
    /// Returns 0 if no data is available (the chip only sent status bytes).
    pub fn read_data(&mut self, buf: &mut [u8]) -> Result<usize> {
        use nusb::transfer::{Bulk, In};

        if buf.is_empty() {
            return Ok(0);
        }

        let packet_size = self.max_packet_size;
        if packet_size == 0 {
            return Err(Error::InvalidArgument("max_packet_size is zero"));
        }

        // Serve from internal buffer first
        if self.readbuffer_remaining > 0 {
            let n = self.readbuffer_remaining.min(buf.len());
            buf[..n].copy_from_slice(
                &self.readbuffer[self.readbuffer_offset..self.readbuffer_offset + n],
            );
            self.readbuffer_remaining -= n;
            self.readbuffer_offset += n;
            return Ok(n);
        }

        // Issue a USB bulk read
        let mut ep = self
            .interface
            .endpoint::<Bulk, In>(self.read_ep)
            .map_err(Error::Usb)?;

        let transfer_buf = nusb::transfer::Buffer::new(self.readbuffer_chunksize);

        let completion = ep.transfer_blocking(transfer_buf, self.read_timeout);
        completion.status.map_err(Error::Transfer)?;

        let actual_length = completion.actual_len;

        if actual_length <= 2 {
            // Only modem status bytes, no payload
            return Ok(0);
        }

        // Copy raw data into our internal buffer for stripping
        let raw_data = completion.buffer.into_vec();
        self.readbuffer[..actual_length].copy_from_slice(&raw_data[..actual_length]);

        // Strip 2-byte modem status from each max_packet_size chunk
        let stripped = strip_modem_status(&mut self.readbuffer[..actual_length], packet_size);

        if stripped == 0 {
            return Ok(0);
        }

        let n = stripped.min(buf.len());
        buf[..n].copy_from_slice(&self.readbuffer[..n]);

        if stripped > buf.len() {
            // Save remainder - shift data to beginning of buffer
            self.readbuffer.copy_within(n..stripped, 0);
            self.readbuffer_offset = 0;
            self.readbuffer_remaining = stripped - n;
        } else {
            self.readbuffer_offset = 0;
            self.readbuffer_remaining = 0;
        }

        Ok(n)
    }

    /// Write all bytes to the device, retrying until complete.
    pub fn write_all(&mut self, buf: &[u8]) -> Result<()> {
        let mut offset = 0;
        while offset < buf.len() {
            let n = self.write_data(&buf[offset..])?;
            if n == 0 {
                return Err(Error::WriteZero);
            }
            offset += n;
        }
        Ok(())
    }
}

/// Strip the 2-byte modem status header from each packet in a raw USB bulk
/// read result. Returns the total number of payload bytes after stripping.
///
/// The data is compacted in-place: payload bytes are moved to fill the
/// gaps left by removed status bytes.
fn strip_modem_status(data: &mut [u8], packet_size: usize) -> usize {
    let total = data.len();
    if total <= 2 {
        return 0;
    }

    let num_packets = total.div_ceil(packet_size);
    let mut write_pos = 0;

    for i in 0..num_packets {
        let pkt_start = i * packet_size;
        let pkt_end = (pkt_start + packet_size).min(total);
        let pkt_len = pkt_end - pkt_start;

        if pkt_len <= 2 {
            // Packet is only status bytes, skip entirely
            continue;
        }

        let payload_start = pkt_start + 2;
        let payload_len = pkt_len - 2;

        if write_pos != payload_start {
            // Need to move data
            data.copy_within(payload_start..payload_start + payload_len, write_pos);
        }
        write_pos += payload_len;
    }

    write_pos
}

/// Determine the maximum packet size for a device.
fn determine_max_packet_size(
    device: &nusb::Device,
    chip_type: ChipType,
    interface_num: u8,
) -> usize {
    // Default based on chip type
    let default_size = if chip_type.is_h_type() { 512 } else { 64 };

    // Try to read from the configuration descriptor
    let config = match device.active_configuration() {
        Ok(c) => c,
        Err(_) => return default_size,
    };

    for iface_group in config.interfaces() {
        if iface_group.interface_number() != interface_num {
            continue;
        }
        for alt in iface_group.alt_settings() {
            if let Some(ep) = alt.endpoints().next() {
                return ep.max_packet_size();
            }
        }
    }

    default_size
}

// ---- std::io trait implementations ----

impl io::Read for FtdiDevice {
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        self.read_data(buf).map_err(io::Error::other)
    }
}

impl io::Write for FtdiDevice {
    fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
        self.write_data(buf).map_err(io::Error::other)
    }

    fn flush(&mut self) -> io::Result<()> {
        self.flush_tx().map_err(io::Error::other)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn strip_modem_status_single_packet() {
        // Simulate a 64-byte packet with 2 status + 62 payload
        let mut data = vec![0u8; 64];
        data[0] = 0x01; // status byte 1
        data[1] = 0x60; // status byte 2
        for (i, byte) in data.iter_mut().enumerate().take(64).skip(2) {
            *byte = i as u8;
        }

        let stripped = strip_modem_status(&mut data, 64);
        assert_eq!(stripped, 62);
        for (i, byte) in data.iter().enumerate().take(62) {
            assert_eq!(*byte, (i + 2) as u8);
        }
    }

    #[test]
    fn strip_modem_status_multiple_packets() {
        let packet_size = 8;
        // Two full packets: [S S 2 3 4 5 6 7] [S S A B C D E F]
        let mut data = vec![
            0xAA, 0xBB, 2, 3, 4, 5, 6, 7, // packet 1
            0xCC, 0xDD, 10, 11, 12, 13, 14, 15, // packet 2
        ];

        let stripped = strip_modem_status(&mut data, packet_size);
        assert_eq!(stripped, 12); // 6 + 6 payload bytes
        assert_eq!(&data[..12], &[2, 3, 4, 5, 6, 7, 10, 11, 12, 13, 14, 15]);
    }

    #[test]
    fn strip_modem_status_short() {
        let mut data = vec![0x01, 0x60]; // Only status bytes
        assert_eq!(strip_modem_status(&mut data, 64), 0);
    }

    #[test]
    fn strip_modem_status_empty() {
        let mut data: Vec<u8> = vec![];
        assert_eq!(strip_modem_status(&mut data, 64), 0);
    }
}
