#[repr(u8)]
#[allow(dead_code)]
pub(crate) enum Request {
    SetTransceiverMode = 1,
    Max2837Write = 2,
    Max2837Read = 3,
    Si5351CWrite = 4,
    Si5351CRead = 5,
    SampleRateSet = 6,
    BasebandFilterBandwidthSet = 7,
    Rffc5071Write = 8,
    Rffc5071Read = 9,
    SpiflashErase = 10,
    SpiflashWrite = 11,
    SpiflashRead = 12,
    BoardIdRead = 14,
    VersionStringRead = 15,
    SetFreq = 16,
    AmpEnable = 17,
    BoardPartidSerialnoRead = 18,
    SetLnaGain = 19,
    SetVgaGain = 20,
    SetTxvgaGain = 21,
    AntennaEnable = 23,
    SetFreqExplicit = 24,
    UsbWcidVendorReq = 25,
    InitSweep = 26,
    OperacakeGetBoards = 27,
    OperacakeSetPorts = 28,
    SetHwSyncMode = 29,
    Reset = 30,
    OperacakeSetRanges = 31,
    ClkoutEnable = 32,
    SpiflashStatus = 33,
    SpiflashClearStatus = 34,
    OperacakeGpioTest = 35,
    CpldChecksum = 36,
    UiEnable = 37,
}

/// Operating modes of the HackRF One.
#[atomic_enum::atomic_enum]
#[derive(PartialEq)]
#[repr(u16)]
pub enum Mode {
    /// Transceiver is off.
    Off = 0,
    /// Transceiver is in receive mode.
    Receive = 1,
    /// Transceiver is in transmit mode.
    Transmit = 2,
    // TODO: HACKRF_TRANSCEIVER_MODE_SS, TRANSCEIVER_MODE_CPLD_UPDATE, TRANSCEIVER_MODE_RX_SWEEP
}

/// Configurable parameters on the hackrf
#[derive(Debug)]
pub struct Config {
    /// Baseband gain, 0-62dB in 2dB increments (rx only)
    pub vga_db: u16,
    /// 0 - 47 dB in 1dB increments (tx only)
    pub txvga_db: u16,

    /// Low-noise amplifier gain, in 0-40dB in 8dB increments (rx only)
    // Pre baseband receive
    pub lna_db: u16,
    /// RF amplifier (on/off)
    pub amp_enable: bool,

    /// Antenna power port control
    // Power enable on antenna
    pub antenna_enable: bool,
    /// Frequency in hz
    pub frequency_hz: u64,
    /// Sample rate in Hz.
    pub sample_rate_hz: u32,
    // TODO: provide helpers for setting this up
    /// Sample rate divider.
    pub sample_rate_div: u32,
}

impl Config {
    /// Returns the default configuration for transmitting.
    pub fn tx_default() -> Self {
        Self {
            vga_db: 0,
            lna_db: 0,
            txvga_db: 40,
            amp_enable: false,
            antenna_enable: false,
            frequency_hz: 908_000_000,
            sample_rate_hz: 2_500_000,
            sample_rate_div: 1,
        }
    }

    /// Returns the default configuration for receiving.
    pub fn rx_default() -> Self {
        Self {
            vga_db: 24,
            lna_db: 0,
            txvga_db: 0,
            amp_enable: false,
            antenna_enable: false,
            frequency_hz: 908_000_000,
            sample_rate_hz: 2_500_000,
            sample_rate_div: 1,
        }
    }
}

/// HackRF One errors.
#[derive(thiserror::Error, Debug)]
pub enum Error {
    /// I/O error occurred.
    #[error("io")]
    Io(#[from] std::io::Error),
    /// USB transfer error.
    #[error("transfer")]
    Transfer(#[from] nusb::transfer::TransferError),
    /// Transfer truncated.
    #[error("transfer truncated")]
    TransferTruncated {
        /// Actual amount of bytes transferred.
        actual: usize,
        /// Expected number of bytes transferred.
        expected: usize,
    },
    /// An API call is not supported by your hardware.
    ///
    /// Try updating the firmware on your device.
    #[error("no api")]
    NoApi {
        /// Current device version.
        device: UsbVersion,
        /// Minimum version required.
        min: UsbVersion,
    },
    /// Invalid argument provided.
    #[error("{0}")]
    Argument(&'static str),
    /// HackRF is in an invalid mode.
    #[error("HackRF in invalid mode. Required: {required:?}, actual: {actual:?}")]
    WrongMode {
        /// The mode required for this operation.
        required: Mode,
        /// The actual mode of the device which differs from `required`.
        actual: Mode,
    },
    /// Device not found.
    #[error("Device not found")]
    NotFound,
}

/// Result type for operations that may return an `Error`.
pub type Result<T> = std::result::Result<T, Error>;

/// A three-part version consisting of major, minor, and sub minor components.
///
/// The intended use case of `Version` is to extract meaning from the version fields in USB
/// descriptors, such as `bcdUSB` and `bcdDevice` in device descriptors.
#[derive(Debug, PartialEq, Eq, Clone, Copy, Hash, PartialOrd, Ord)]
// Taken from rusb::Version: https://github.com/a1ien/rusb/blob/8f8c3c6bff6a494a140da4d93dd946bf1e564d66/src/fields.rs#L142-L203
// nusb doesnt currently have this
pub struct UsbVersion(pub u8, pub u8, pub u8);

impl UsbVersion {
    /// Extracts a version from a binary coded decimal (BCD) field. BCD fields exist in USB
    /// descriptors as 16-bit integers encoding a version as `0xJJMN`, where `JJ` is the major
    /// version, `M` is the minor version, and `N` is the sub minor version. For example, 2.0 is
    /// encoded as `0x0200` and 1.1 is encoded as `0x0110`.
    pub fn from_bcd(mut raw: u16) -> Self {
        let sub_minor: u8 = (raw & 0x000F) as u8;
        raw >>= 4;

        let minor: u8 = (raw & 0x000F) as u8;
        raw >>= 4;

        let mut major: u8 = (raw & 0x000F) as u8;
        raw >>= 4;

        major += (10 * raw) as u8;

        UsbVersion(major, minor, sub_minor)
    }

    /// Returns the major version.
    pub fn major(self) -> u8 {
        let UsbVersion(major, _, _) = self;
        major
    }

    /// Returns the minor version.
    pub fn minor(self) -> u8 {
        let UsbVersion(_, minor, _) = self;
        minor
    }

    /// Returns the sub minor version.
    pub fn sub_minor(self) -> u8 {
        let UsbVersion(_, _, sub_minor) = self;
        sub_minor
    }
}

impl std::fmt::Display for UsbVersion {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}.{}.{}", self.major(), self.minor(), self.sub_minor())
    }
}
