//! Type definitions for FTDI chip communication.
//!
//! These types model the various configuration options for FTDI chips:
//! chip variants, serial line properties, bitbang modes, and modem status.

/// Supported FTDI chip types.
///
/// The chip type is auto-detected when a device is opened, based on the
/// USB `bcdDevice` descriptor field.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum ChipType {
    /// Original FTDI chip (FT8U232AM).
    Am,
    /// B-type chip (FT232BM, FT245BM).
    Bm,
    /// Dual-port chip (FT2232C/D/L).
    Ft2232C,
    /// FT232R / FT245R.
    Ft232R,
    /// Dual hi-speed chip (FT2232H).
    Ft2232H,
    /// Quad-port chip (FT4232H).
    Ft4232H,
    /// Single hi-speed chip (FT232H).
    Ft232H,
    /// FT230X / FT231X / FT234XD.
    Ft230X,
}

impl ChipType {
    /// Whether this is an H-type (hi-speed) chip.
    #[inline]
    pub fn is_h_type(self) -> bool {
        matches!(self, Self::Ft2232H | Self::Ft4232H | Self::Ft232H)
    }

    /// Default product string for this chip type.
    pub fn default_product_name(self) -> &'static str {
        match self {
            Self::Am => "AM",
            Self::Bm => "BM",
            Self::Ft2232C => "Dual RS232",
            Self::Ft232R => "FT232R USB UART",
            Self::Ft2232H => "Dual RS232-HS",
            Self::Ft4232H => "FT4232H",
            Self::Ft232H => "Single-RS232-HS",
            Self::Ft230X => "FT230X Basic UART",
        }
    }

    /// Default product ID for this chip type.
    pub fn default_product_id(self) -> u16 {
        match self {
            Self::Am | Self::Bm | Self::Ft232R => 0x6001,
            Self::Ft2232C | Self::Ft2232H => 0x6010,
            Self::Ft4232H => 0x6011,
            Self::Ft232H => 0x6014,
            Self::Ft230X => 0x6015,
        }
    }

    /// Default USB release number (bcdDevice) for this chip type.
    pub fn release_number(self) -> u16 {
        match self {
            Self::Am => 0x0200,
            Self::Bm => 0x0400,
            Self::Ft2232C => 0x0500,
            Self::Ft232R => 0x0600,
            Self::Ft2232H => 0x0700,
            Self::Ft4232H => 0x0800,
            Self::Ft232H => 0x0900,
            Self::Ft230X => 0x1000,
        }
    }
}

/// Parity mode for serial communication.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum Parity {
    /// No parity bit.
    #[default]
    None,
    /// Odd parity.
    Odd,
    /// Even parity.
    Even,
    /// Mark parity (always 1).
    Mark,
    /// Space parity (always 0).
    Space,
}

impl Parity {
    /// Wire encoding for the SIO_SET_DATA request.
    pub(crate) fn wire_value(self) -> u16 {
        match self {
            Self::None => 0x00,
            Self::Odd => 0x01,
            Self::Even => 0x02,
            Self::Mark => 0x03,
            Self::Space => 0x04,
        }
    }
}

/// Number of stop bits for serial communication.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum StopBits {
    /// 1 stop bit.
    #[default]
    One,
    /// 1.5 stop bits.
    OnePointFive,
    /// 2 stop bits.
    Two,
}

impl StopBits {
    /// Wire encoding for the SIO_SET_DATA request.
    pub(crate) fn wire_value(self) -> u16 {
        match self {
            Self::One => 0x00,
            Self::OnePointFive => 0x01,
            Self::Two => 0x02,
        }
    }
}

/// Number of data bits for serial communication.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum DataBits {
    /// 7 data bits.
    Seven,
    /// 8 data bits.
    #[default]
    Eight,
}

impl DataBits {
    /// Wire encoding for the SIO_SET_DATA request.
    pub(crate) fn wire_value(self) -> u16 {
        match self {
            Self::Seven => 7,
            Self::Eight => 8,
        }
    }
}

/// Break signal state.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum BreakType {
    /// Break off (normal operation).
    #[default]
    Off,
    /// Break on (hold TX low).
    On,
}

impl BreakType {
    /// Wire encoding for the SIO_SET_DATA request.
    pub(crate) fn wire_value(self) -> u16 {
        match self {
            Self::Off => 0x00,
            Self::On => 0x01,
        }
    }
}

/// Bitbang / MPSSE mode selection.
///
/// Used with [`FtdiDevice::set_bitmode`](crate::FtdiDevice::set_bitmode).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum BitMode {
    /// Normal serial/FIFO mode (bitbang disabled).
    #[default]
    Reset,
    /// Asynchronous bitbang mode (B-type and later).
    BitBang,
    /// MPSSE mode (FT2232x and later).
    Mpsse,
    /// Synchronous bitbang mode (FT2232x, FT232R and later).
    SyncBB,
    /// MCU host bus emulation mode (FT2232x).
    Mcu,
    /// Fast opto-isolated serial mode (FT2232x).
    Opto,
    /// CBUS bitbang mode (FT232R, configure in EEPROM first).
    Cbus,
    /// Synchronous FIFO mode (FT2232H).
    SyncFf,
    /// FT1284 mode (FT232H).
    Ft1284,
}

impl BitMode {
    /// Wire value for the SIO_SET_BITMODE request.
    pub(crate) fn wire_value(self) -> u8 {
        match self {
            Self::Reset => 0x00,
            Self::BitBang => 0x01,
            Self::Mpsse => 0x02,
            Self::SyncBB => 0x04,
            Self::Mcu => 0x08,
            Self::Opto => 0x10,
            Self::Cbus => 0x20,
            Self::SyncFf => 0x40,
            Self::Ft1284 => 0x80,
        }
    }
}

/// Port interface selection for multi-interface chips.
///
/// Chips like the FT2232H (dual) and FT4232H (quad) expose multiple
/// independent interfaces. Select which one to use before opening.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum Interface {
    /// Use the first available interface (same as `A`).
    #[default]
    Any,
    /// Interface A (port 0).
    A,
    /// Interface B (port 1).
    B,
    /// Interface C (port 2, FT4232H only).
    C,
    /// Interface D (port 3, FT4232H only).
    D,
}

/// Kernel module detach behavior when opening a device.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum ModuleDetachMode {
    /// Automatically detach the kernel module (e.g. `ftdi_sio`).
    #[default]
    AutoDetach,
    /// Do not detach the kernel module.
    DontDetach,
    /// Detach on open, re-attach on close.
    AutoDetachReattach,
}

/// Flow control mode.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum FlowControl {
    /// No flow control.
    #[default]
    Disabled,
    /// Hardware RTS/CTS flow control.
    RtsCts,
    /// Hardware DTR/DSR flow control.
    DtrDsr,
}

/// Decoded modem status from the FTDI chip.
///
/// The FTDI chip sends two status bytes as a header with every USB read.
/// This struct represents the decoded content.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ModemStatus {
    raw: u16,
}

impl ModemStatus {
    /// Create from the raw two-byte status value.
    pub(crate) fn from_raw(raw: u16) -> Self {
        Self { raw }
    }

    /// Raw 16-bit status value.
    pub fn raw(self) -> u16 {
        self.raw
    }

    // -- Byte 0 (modem status lines) --

    /// Clear To Send (CTS) is active.
    pub fn cts(self) -> bool {
        self.raw & 0x10 != 0
    }

    /// Data Set Ready (DSR) is active.
    pub fn dsr(self) -> bool {
        self.raw & 0x20 != 0
    }

    /// Ring Indicator (RI) is active.
    pub fn ri(self) -> bool {
        self.raw & 0x40 != 0
    }

    /// Receive Line Signal Detect (RLSD / DCD) is active.
    pub fn rlsd(self) -> bool {
        self.raw & 0x80 != 0
    }

    // -- Byte 1 (line status) --

    /// Data Ready (DR).
    pub fn data_ready(self) -> bool {
        self.raw & 0x0100 != 0
    }

    /// Overrun Error (OE).
    pub fn overrun_error(self) -> bool {
        self.raw & 0x0200 != 0
    }

    /// Parity Error (PE).
    pub fn parity_error(self) -> bool {
        self.raw & 0x0400 != 0
    }

    /// Framing Error (FE).
    pub fn framing_error(self) -> bool {
        self.raw & 0x0800 != 0
    }

    /// Break Interrupt (BI).
    pub fn break_interrupt(self) -> bool {
        self.raw & 0x1000 != 0
    }

    /// Transmitter Holding Register Empty (THRE).
    pub fn transmitter_holding_empty(self) -> bool {
        self.raw & 0x2000 != 0
    }

    /// Transmitter Empty (TEMT).
    pub fn transmitter_empty(self) -> bool {
        self.raw & 0x4000 != 0
    }

    /// Error in RCVR FIFO.
    pub fn fifo_error(self) -> bool {
        self.raw & 0x8000 != 0
    }
}

/// Interface configuration resolved to concrete USB endpoint values.
#[derive(Debug, Clone, Copy)]
pub(crate) struct InterfaceConfig {
    /// The USB interface number (0-based).
    pub interface_num: u8,
    /// The USB index value used in control transfers (1-based).
    pub usb_index: u16,
    /// The bulk OUT endpoint address (host-to-device, for writing data).
    pub write_ep: u8,
    /// The bulk IN endpoint address (device-to-host, for reading data).
    pub read_ep: u8,
}

impl Interface {
    /// Resolve to concrete USB endpoint configuration.
    pub(crate) fn config(self) -> InterfaceConfig {
        match self {
            Self::Any | Self::A => InterfaceConfig {
                interface_num: 0,
                usb_index: 1,
                write_ep: 0x02,
                read_ep: 0x81,
            },
            Self::B => InterfaceConfig {
                interface_num: 1,
                usb_index: 2,
                write_ep: 0x04,
                read_ep: 0x83,
            },
            Self::C => InterfaceConfig {
                interface_num: 2,
                usb_index: 3,
                write_ep: 0x06,
                read_ep: 0x85,
            },
            Self::D => InterfaceConfig {
                interface_num: 3,
                usb_index: 4,
                write_ep: 0x08,
                read_ep: 0x87,
            },
        }
    }
}
