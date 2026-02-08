//! EEPROM data types and structures.

use crate::constants::FTDI_MAX_EEPROM_SIZE;

/// Decoded FTDI EEPROM contents.
///
/// This structure mirrors the logical fields stored in the FTDI chip's
/// EEPROM (up to 256 bytes). Fields are populated by [`decode`](super::decode)
/// and consumed by [`build`](super::build).
#[derive(Debug, Clone)]
pub struct FtdiEeprom {
    // ---- Identification ----
    /// USB Vendor ID (default: 0x0403).
    pub vendor_id: u16,
    /// USB Product ID.
    pub product_id: u16,
    /// Device release number (bcdDevice).
    pub release_number: u16,

    /// Whether the EEPROM was initialized for the currently connected device.
    pub initialized_for_connected_device: bool,

    // ---- Power / USB configuration ----
    /// Device is self-powered (vs bus-powered).
    pub self_powered: bool,
    /// Device supports USB remote wakeup.
    pub remote_wakeup: bool,
    /// Device is not Plug-and-Play.
    pub is_not_pnp: bool,
    /// Suspend on DBUS7 low (FT2232H).
    pub suspend_dbus7: bool,
    /// Input endpoint is isochronous.
    pub in_is_isochronous: bool,
    /// Output endpoint is isochronous.
    pub out_is_isochronous: bool,
    /// Pull down pins during suspend.
    pub suspend_pull_downs: bool,
    /// Use the serial number string.
    pub use_serial: bool,
    /// USB version (bcdUSB).
    pub usb_version: u16,
    /// Use explicit USB version (BM/2232C).
    pub use_usb_version: bool,
    /// Maximum power consumption in mA.
    pub max_power: u16,

    // ---- String descriptors ----
    /// Manufacturer name.
    pub manufacturer: Option<String>,
    /// Product description.
    pub product: Option<String>,
    /// Serial number.
    pub serial: Option<String>,

    // ---- Channel configuration (multi-interface chips) ----
    /// Channel A hardware type (UART/FIFO/OPTO/CPU/FT1284).
    pub channel_a_type: u8,
    /// Channel B hardware type.
    pub channel_b_type: u8,
    /// Channel A uses VCP driver.
    pub channel_a_driver: bool,
    /// Channel B uses VCP driver.
    pub channel_b_driver: bool,
    /// Channel C uses VCP driver (FT4232H).
    pub channel_c_driver: bool,
    /// Channel D uses VCP driver (FT4232H).
    pub channel_d_driver: bool,
    /// Channel A RS485 enable (FT4232H).
    pub channel_a_rs485enable: bool,
    /// Channel B RS485 enable (FT4232H).
    pub channel_b_rs485enable: bool,
    /// Channel C RS485 enable (FT4232H).
    pub channel_c_rs485enable: bool,
    /// Channel D RS485 enable (FT4232H).
    pub channel_d_rs485enable: bool,

    // ---- CBUS pin functions ----
    /// CBUS pin function assignments (up to 10 pins).
    pub cbus_function: [u8; 10],

    // ---- Current / IO configuration ----
    /// High current drive (FT232R).
    pub high_current: bool,
    /// High current drive on channel A (FT2232C).
    pub high_current_a: bool,
    /// High current drive on channel B (FT2232C).
    pub high_current_b: bool,
    /// Signal inversion bitmask.
    pub invert: u8,
    /// External oscillator (FT232R).
    pub external_oscillator: bool,

    // ---- IO group drive/schmitt/slew (H-type chips) ----
    pub group0_drive: u8,
    pub group0_schmitt: bool,
    pub group0_slew: bool,
    pub group1_drive: u8,
    pub group1_schmitt: bool,
    pub group1_slew: bool,
    pub group2_drive: u8,
    pub group2_schmitt: bool,
    pub group2_slew: bool,
    pub group3_drive: u8,
    pub group3_schmitt: bool,
    pub group3_slew: bool,

    // ---- FT232H specific ----
    /// Power save mode (FT232H).
    pub powersave: bool,
    /// FT1284 clock polarity.
    pub clock_polarity: bool,
    /// FT1284 data order (LSB first).
    pub data_order: bool,
    /// FT1284 flow control.
    pub flow_control: bool,

    // ---- User data ----
    /// Start address of user data area.
    pub user_data_addr: usize,
    /// Size of user data.
    pub user_data_size: usize,
    /// User data content.
    pub user_data: Option<Vec<u8>>,

    // ---- Internal ----
    /// EEPROM size in bytes (128, 256, or -1 for unknown).
    pub size: i32,
    /// EEPROM chip type (0x46=93x46, 0x56=93x56, 0x66=93x66, 0=internal, -1=unknown).
    pub chip: i32,
    /// Raw EEPROM binary content.
    pub buf: [u8; FTDI_MAX_EEPROM_SIZE],
}

impl Default for FtdiEeprom {
    fn default() -> Self {
        Self {
            vendor_id: 0x0403,
            product_id: 0x6001,
            release_number: 0,
            initialized_for_connected_device: false,
            self_powered: false,
            remote_wakeup: false,
            is_not_pnp: false,
            suspend_dbus7: false,
            in_is_isochronous: false,
            out_is_isochronous: false,
            suspend_pull_downs: false,
            use_serial: true,
            usb_version: 0x0200,
            use_usb_version: false,
            max_power: 100,
            manufacturer: None,
            product: None,
            serial: None,
            channel_a_type: 0,
            channel_b_type: 0,
            channel_a_driver: false,
            channel_b_driver: false,
            channel_c_driver: false,
            channel_d_driver: false,
            channel_a_rs485enable: false,
            channel_b_rs485enable: false,
            channel_c_rs485enable: false,
            channel_d_rs485enable: false,
            cbus_function: [0; 10],
            high_current: false,
            high_current_a: false,
            high_current_b: false,
            invert: 0,
            external_oscillator: false,
            group0_drive: 0,
            group0_schmitt: false,
            group0_slew: false,
            group1_drive: 0,
            group1_schmitt: false,
            group1_slew: false,
            group2_drive: 0,
            group2_schmitt: false,
            group2_slew: false,
            group3_drive: 0,
            group3_schmitt: false,
            group3_slew: false,
            powersave: false,
            clock_polarity: false,
            data_order: false,
            flow_control: false,
            user_data_addr: 0,
            user_data_size: 0,
            user_data: None,
            size: -1,
            chip: -1,
            buf: [0u8; FTDI_MAX_EEPROM_SIZE],
        }
    }
}

impl FtdiEeprom {
    /// Get the raw EEPROM buffer.
    pub fn raw_buf(&self) -> &[u8; FTDI_MAX_EEPROM_SIZE] {
        &self.buf
    }

    /// Set the raw EEPROM buffer from a slice.
    ///
    /// Only copies up to `FTDI_MAX_EEPROM_SIZE` bytes.
    pub fn set_raw_buf(&mut self, data: &[u8]) {
        let len = data.len().min(FTDI_MAX_EEPROM_SIZE);
        self.buf[..len].copy_from_slice(&data[..len]);
    }

    /// Get the EEPROM strings as a tuple of optional references.
    pub fn strings(&self) -> (Option<&str>, Option<&str>, Option<&str>) {
        (
            self.manufacturer.as_deref(),
            self.product.as_deref(),
            self.serial.as_deref(),
        )
    }

    /// Set the EEPROM string descriptors.
    pub fn set_strings(
        &mut self,
        manufacturer: Option<&str>,
        product: Option<&str>,
        serial: Option<&str>,
    ) {
        if let Some(m) = manufacturer {
            self.manufacturer = Some(m.to_owned());
        }
        if let Some(p) = product {
            self.product = Some(p.to_owned());
        }
        if let Some(s) = serial {
            self.serial = Some(s.to_owned());
            self.use_serial = true;
        }
    }
}
