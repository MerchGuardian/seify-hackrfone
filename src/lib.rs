#![deny(unsafe_code)]

//! # HackRF One API
//!
//! This crate provides a Rust interface to the [HackRF One](https://greatscottgadgets.com/hackrf/one/),
//! a popular software-defined radio (SDR) peripheral. It allows for transmitting and receiving
//! radio signals using the HackRF One device in pure Rust.
//!
//! ## Example
//!
//! ```rust,no_run
//! use anyhow::Result;
//! use seify_hackrfone::{Config, HackRf};
//!
//! fn main() -> Result<()> {
//!     let radio = HackRf::open_first()?;
//!
//!     radio.start_rx(&Config {
//!         vga_db: 0,
//!         txvga_db: 0,
//!         lna_db: 0,
//!         amp_enable: false,
//!         antenna_enable: false,
//!         frequency_hz: 915_000_000,
//!         sample_rate_hz: 2_000_000,
//!         sample_rate_div: 1,
//!     })?;
//!
//!     let mut buf = vec![0u8; 32 * 1024];
//!     loop {
//!         radio.read(&mut buf)?;
//!         // Process samples...
//!     }
//! }
//! ```
//!
//! ## License
//!
//! This crate is licensed under the MIT License.

#![cfg_attr(docsrs, feature(doc_cfg), feature(doc_auto_cfg))]
// TODO(tjn): re-enable
#![warn(missing_docs)]

mod types;
pub use types::*;

use std::sync::atomic::Ordering;

use futures_lite::future::block_on;
use nusb::{
    transfer::{ControlIn, ControlOut, ControlType, Queue, Recipient, RequestBuffer},
    DeviceInfo,
};

/// HackRF USB vendor ID.
const HACKRF_USB_VID: u16 = 0x1D50;
/// HackRF One USB product ID.
const HACKRF_ONE_USB_PID: u16 = 0x6089;

/// HackRF One software defined radio.
pub struct HackRf {
    interface: nusb::Interface,
    version: UsbVersion,
    mode: AtomicMode,
}

impl HackRf {
    /// Opens `info` based on the result of a `nusb` scan.
    pub fn open(info: DeviceInfo) -> Result<Self> {
        let device = info.open()?;

        let interface = device
            .detach_and_claim_interface(0)
            .expect("claim interface");

        Ok(HackRf {
            interface,
            version: UsbVersion::from_bcd(info.device_version()),
            mode: AtomicMode::new(Mode::Off),
        })
    }

    /// Wraps a HackRf One exposed through an existing file descriptor.
    ///
    /// Useful on platforms like Android where [`UsbManager`](https://developer.android.com/reference/android/hardware/usb/UsbManager#openAccessory(android.hardware.usb.UsbAccessory)) permits access to an fd.
    #[cfg(any(target_os = "android", target_os = "linux"))]
    pub fn from_fd(fd: std::os::fd::OwnedFd) -> Result<Self> {
        use std::os::fd::AsRawFd;
        log::info!("Wrapping hackrf fd={}", fd.as_raw_fd());
        let device = nusb::Device::from_fd(fd)?;

        let interface = device
            .detach_and_claim_interface(0)
            .expect("claim interface");

        Ok(HackRf {
            interface,
            // TODO: Actually read version, dont assume latest
            version: UsbVersion::from_bcd(0x0102),
            mode: AtomicMode::new(Mode::Off),
        })
    }

    /// Opens the first Hackrf One found via USB.
    pub fn open_first() -> Result<HackRf> {
        for device in nusb::list_devices()? {
            if device.vendor_id() == HACKRF_USB_VID && device.product_id() == HACKRF_ONE_USB_PID {
                match Self::open(device) {
                    Ok(dev) => return Ok(dev),
                    Err(_) => continue,
                }
            }
        }

        Err(Error::NotFound)
    }

    /// Scans the usb bus for hackrf devices, returning the pair of (bus_num, bus_addr) for each
    /// device.
    // TODO: add equivalent macos function that uses location_id
    #[cfg(any(target_os = "linux", target_os = "android"))]
    pub fn scan() -> Result<Vec<(u8, u8)>> {
        let mut res = vec![];
        for device in nusb::list_devices()? {
            if device.vendor_id() == HACKRF_USB_VID && device.product_id() == HACKRF_ONE_USB_PID {
                res.push((device.bus_number(), device.device_address()));
            }
        }
        Ok(res)
    }

    /// Opens a hackrf with usb address `<bus_number>:<address>`
    // TODO: add equivalent macos function that uses location_id
    #[cfg(any(target_os = "linux", target_os = "android"))]
    pub fn open_bus(bus_number: u8, address: u8) -> Result<HackRf> {
        for device in nusb::list_devices()? {
            match device.vendor_id() == HACKRF_USB_VID
                && device.product_id() == HACKRF_ONE_USB_PID
                && device.bus_number() == bus_number
                && device.device_address() == address
            {
                true => return Self::open(device),
                false => (),
            }
        }

        Err(Error::NotFound)
    }

    /// Resets the HackRf.
    pub fn reset(self) -> Result<()> {
        self.check_api_version(UsbVersion::from_bcd(0x0102))?;
        self.write_control(Request::Reset, 0, 0, &[])?;

        Ok(())
    }

    /// Returns the USB version of the device.
    pub fn device_version(&self) -> UsbVersion {
        self.version
    }

    /// Reads the board ID of the HackRF One device.
    pub fn board_id(&self) -> Result<u8> {
        let data: [u8; 1] = self.read_control(Request::BoardIdRead, 0, 0)?;
        Ok(data[0])
    }

    /// Read the firmware version.
    pub fn version(&self) -> Result<String> {
        let buf = block_on(self.interface.control_in(ControlIn {
            control_type: ControlType::Vendor,
            recipient: Recipient::Device,
            request: Request::VersionStringRead as u8,
            value: 0x0,
            index: 0x0,
            length: 64,
        }))
        .into_result()?;

        Ok(String::from_utf8_lossy(&buf).into())
    }

    fn apply_config(&self, config: &Config) -> Result<()> {
        self.set_lna_gain(config.lna_db)?;
        self.set_vga_gain(config.vga_db)?;
        self.set_txvga_gain(config.txvga_db)?;
        self.set_freq(config.frequency_hz)?;
        self.set_amp_enable(config.amp_enable)?;
        self.set_antenna_enable(config.antenna_enable)?;
        self.set_sample_rate(config.sample_rate_hz, config.sample_rate_div)?;

        Ok(())
    }

    /// Transitions the radio into transmit mode.
    /// Call this function before calling [`Self::write`].
    ///
    /// Previous state set via `set_xxx` functions will be overridden with the parameters set in `config`.
    ///
    /// # Errors
    /// This function will return an error if a tx or rx operation is already in progress or if an
    /// I/O error occurs
    pub fn start_tx(&self, config: &Config) -> Result<()> {
        // NOTE: perform atomic exchange first so that we only change the transceiver mode once if
        // other threads are racing to change with us
        if let Err(actual) = self.mode.compare_exchange(
            Mode::Off,
            Mode::Transmit,
            Ordering::AcqRel,
            Ordering::Relaxed,
        ) {
            return Err(Error::WrongMode {
                required: Mode::Off,
                actual,
            });
        }

        self.apply_config(config)?;

        self.write_control(Request::SetTransceiverMode, Mode::Transmit as u16, 0, &[])?;

        Ok(())
    }

    /// Transitions the radio into receive mode.
    /// Call this function before calling [`Self::read`].
    ///
    /// Previous state set via `set_xxx` functions will be overridden with the parameters set in `config`.
    ///
    /// # Errors
    /// This function will return an error if a tx or rx operation is already in progress or if an
    /// I/O error occurs
    pub fn start_rx(&self, config: &Config) -> Result<()> {
        // NOTE: perform atomic exchange first so that we only change the transceiver mode once if
        // other threads are racing to change with us
        if let Err(actual) = self.mode.compare_exchange(
            Mode::Off,
            Mode::Receive,
            Ordering::AcqRel,
            Ordering::Relaxed,
        ) {
            return Err(Error::WrongMode {
                required: Mode::Off,
                actual,
            });
        }

        self.apply_config(config)?;

        self.write_control(Request::SetTransceiverMode, Mode::Receive as u16, 0, &[])?;

        Ok(())
    }

    /// Stops the transmit operation and transitions the radio into off mode.
    ///
    /// # Errors
    /// This function will return an error if the device is not in transmit mode or if an
    /// I/O error occurs.
    pub fn stop_tx(&self) -> Result<()> {
        // NOTE:  perform atomic exchange last so that we prevent other threads from racing to
        // start tx/rx with the delivery of our TransceiverMode::Off request
        //
        // This means if multiple threads call stop_tx/stop_rx concurrently the hackrf may receive
        // multiple TransceiverMode::Off requests, but will always end up in a valid state with the
        // transceiver disabled.
        //
        // Adding something like Mode::IdlePending would solve this,
        // however quickly this begins to look like a manually implemented mutex.
        //
        // To keep this crate low-level and low-overhead, this solution is fine and we expect
        // consumers to wrap our type in an Arc and be smart enough to not enable / disable tx / rx
        // from multiple threads at the same time on a single duplex radio.

        self.write_control(Request::SetTransceiverMode, Mode::Off as u16, 0, &[])?;

        if let Err(actual) = self.mode.compare_exchange(
            Mode::Transmit,
            Mode::Off,
            Ordering::AcqRel,
            Ordering::Relaxed,
        ) {
            return Err(Error::WrongMode {
                required: Mode::Transmit,
                actual,
            });
        }

        Ok(())
    }

    /// Stops the receive operation and transitions the radio into off mode.
    ///
    /// # Errors
    /// This function will return an error if the device is not in receive mode or if an
    /// I/O error occurs.
    pub fn stop_rx(&self) -> Result<()> {
        // NOTE: same as above - perform atomic exchange last

        self.write_control(Request::SetTransceiverMode, Mode::Off as u16, 0, &[])?;

        if let Err(actual) = self.mode.compare_exchange(
            Mode::Receive,
            Mode::Off,
            Ordering::AcqRel,
            Ordering::Relaxed,
        ) {
            return Err(Error::WrongMode {
                required: Mode::Receive,
                actual,
            });
        }

        Ok(())
    }

    /// Read samples from the radio.
    ///
    /// # Panics
    /// This function panics if samples is not a multiple of 512
    pub fn read(&self, samples: &mut [u8]) -> Result<usize> {
        self.ensure_mode(Mode::Receive)?;

        if samples.len() % 512 != 0 {
            panic!("samples must be a multiple of 512");
        }

        const ENDPOINT: u8 = 0x81;
        let buf = block_on(
            self.interface
                .bulk_in(ENDPOINT, RequestBuffer::new(samples.len())),
        )
        .into_result()?;
        samples[..buf.len()].copy_from_slice(&buf);

        Ok(buf.len())
    }

    /// Writes samples to the radio.
    ///
    /// # Panics
    /// This function panics if samples is not a multiple of 512
    pub fn write(&self, samples: &[u8]) -> Result<usize> {
        self.ensure_mode(Mode::Transmit)?;

        if samples.len() % 512 != 0 {
            panic!("samples must be a multiple of 512");
        }

        const ENDPOINT: u8 = 0x02;
        let buf = Vec::from(samples);
        // TODO: dont allocate
        let n = block_on(self.interface.bulk_out(ENDPOINT, buf)).into_result()?;

        Ok(n.actual_length())
    }

    /// Setup the device to stream samples.
    pub fn start_rx_stream(&self, transfer_size: usize) -> Result<RxStream> {
        if transfer_size % 512 != 0 {
            panic!("transfer_size must be a multiple of 512");
        }

        const ENDPOINT: u8 = 0x81;
        Ok(RxStream {
            queue: self.interface.bulk_in_queue(ENDPOINT),
            in_flight_transfers: 3,
            transfer_size,
            buf_pos: transfer_size,
            buf: vec![0u8; transfer_size],
        })
    }
}

/// Represents an asynchronous receive stream from the HackRF device.
///
/// Use this to read samples from the device in a streaming fashion.
pub struct RxStream {
    queue: Queue<RequestBuffer>,
    in_flight_transfers: usize,
    transfer_size: usize,
    buf_pos: usize,
    buf: Vec<u8>,
}

impl RxStream {
    /// Read samples from the device, blocking until more are available.
    pub fn read_sync(&mut self, count: usize) -> Result<&[u8]> {
        let buffered_remaining = self.buf.len() - self.buf_pos;
        if buffered_remaining > 0 {
            let to_consume = std::cmp::min(count, buffered_remaining);
            let ret = &self.buf[self.buf_pos..self.buf_pos + to_consume];
            self.buf_pos += ret.len();
            return Ok(ret);
        }

        while self.queue.pending() < self.in_flight_transfers {
            self.queue.submit(RequestBuffer::new(self.transfer_size));
        }
        let completion = block_on(self.queue.next_complete());

        if let Err(e) = completion.status {
            return Err(e.into());
        }

        let reuse = std::mem::replace(&mut self.buf, completion.data);
        self.buf_pos = 0;

        self.queue
            .submit(RequestBuffer::reuse(reuse, self.transfer_size));

        // bytes are now buffered, use tail recursion for code above to return subslice
        self.read_sync(count)
    }
}

impl HackRf {
    fn ensure_mode(&self, expected: Mode) -> Result<()> {
        let actual = self.mode.load(Ordering::Acquire);
        if actual != expected {
            return Err(Error::WrongMode {
                required: expected,
                actual,
            });
        }
        Ok(())
    }

    fn read_control<const N: usize>(
        &self,
        request: Request,
        value: u16,
        index: u16,
    ) -> Result<[u8; N]> {
        let mut res: [u8; N] = [0; N];
        let buf = block_on(self.interface.control_in(ControlIn {
            control_type: ControlType::Vendor,
            recipient: Recipient::Device,
            request: request as u8,
            value,
            index,
            length: N as u16,
        }))
        .into_result()?;

        if buf.len() != N {
            return Err(Error::TransferTruncated {
                actual: buf.len(),
                expected: N,
            });
        }

        res.copy_from_slice(&buf);
        Ok(res)
    }

    fn write_control(&self, request: Request, value: u16, index: u16, buf: &[u8]) -> Result<()> {
        let out = block_on(self.interface.control_out(ControlOut {
            control_type: ControlType::Vendor,
            recipient: Recipient::Device,
            request: request as u8,
            value,
            index,
            data: buf,
        }))
        .into_result()?;

        if out.actual_length() != buf.len() {
            Err(Error::TransferTruncated {
                actual: out.actual_length(),
                expected: buf.len(),
            })
        } else {
            Ok(())
        }
    }

    fn check_api_version(&self, min: UsbVersion) -> Result<()> {
        fn version_to_u32(v: UsbVersion) -> u32 {
            ((v.major() as u32) << 16) | ((v.minor() as u32) << 8) | (v.sub_minor() as u32)
        }

        if version_to_u32(self.version) >= version_to_u32(min) {
            Ok(())
        } else {
            Err(Error::NoApi {
                device: self.version,
                min,
            })
        }
    }

    /// Set the center frequency.
    pub fn set_freq(&self, hz: u64) -> Result<()> {
        let buf: [u8; 8] = freq_params(hz);
        self.write_control(Request::SetFreq, 0, 0, &buf)
    }

    /// Enable the RX/TX RF amplifier.
    ///
    /// In GNU radio this is used as the RF gain, where a value of 0 dB is off,
    /// and a value of 14 dB is on.
    pub fn set_amp_enable(&self, enable: bool) -> Result<()> {
        self.write_control(Request::AmpEnable, enable.into(), 0, &[])
    }

    /// Set the baseband filter bandwidth.
    ///
    /// This is automatically set when the sample rate is changed with
    /// [`Self::set_sample_rate`].
    pub fn set_baseband_filter_bandwidth(&self, hz: u32) -> Result<()> {
        self.write_control(
            Request::BasebandFilterBandwidthSet,
            (hz & 0xFFFF) as u16,
            (hz >> 16) as u16,
            &[],
        )
    }

    /// Set the sample rate.
    ///
    /// For anti-aliasing, the baseband filter bandwidth is automatically set to
    /// the widest available setting that is no more than 75% of the sample rate.
    /// This happens every time the sample rate is set.
    /// If you want to override the baseband filter selection, you must do so
    /// after setting the sample rate.
    ///
    /// Limits are 8MHz - 20MHz.
    /// Preferred rates are 8, 10, 12.5, 16, 20MHz due to less jitter.
    pub fn set_sample_rate(&self, hz: u32, div: u32) -> Result<()> {
        let hz: u32 = hz.to_le();
        let div: u32 = div.to_le();
        let buf: [u8; 8] = [
            (hz & 0xFF) as u8,
            ((hz >> 8) & 0xFF) as u8,
            ((hz >> 16) & 0xFF) as u8,
            ((hz >> 24) & 0xFF) as u8,
            (div & 0xFF) as u8,
            ((div >> 8) & 0xFF) as u8,
            ((div >> 16) & 0xFF) as u8,
            ((div >> 24) & 0xFF) as u8,
        ];
        self.write_control(Request::SampleRateSet, 0, 0, &buf)?;
        self.set_baseband_filter_bandwidth((0.75 * (hz as f32) / (div as f32)) as u32)
    }

    /// Set the LNA (low noise amplifier) gain.
    ///
    /// Range 0 to 40dB in 8dB steps.
    ///
    /// This is also known as the IF gain.
    pub fn set_lna_gain(&self, gain: u16) -> Result<()> {
        if gain > 40 {
            Err(Error::Argument("lna gain must be less than 40"))
        } else {
            let buf: [u8; 1] = self.read_control(Request::SetLnaGain, 0, gain & !0x07)?;
            if buf[0] == 0 {
                panic!("Unexpected return value from read_control(SetLnaGain)");
            } else {
                Ok(())
            }
        }
    }

    /// Set the VGA (variable gain amplifier) gain.
    ///
    /// Range 0 to 62dB in 2dB steps.
    ///
    /// This is also known as the baseband (BB) gain.
    pub fn set_vga_gain(&self, gain: u16) -> Result<()> {
        if gain > 62 || gain % 2 == 1 {
            Err(Error::Argument(
                "gain parameter out of range. must be even and less than or equal to 62",
            ))
        } else {
            let buf: [u8; 1] = self.read_control(Request::SetVgaGain, 0, gain & !0b1)?;
            if buf[0] == 0 {
                panic!("What is this return value?")
            } else {
                Ok(())
            }
        }
    }

    /// Set the transmit VGA gain.
    ///
    /// Range 0 to 47dB in 1db steps.
    pub fn set_txvga_gain(&self, gain: u16) -> Result<()> {
        if gain > 47 {
            Err(Error::Argument("gain parameter out of range. max is 47"))
        } else {
            let buf: [u8; 1] = self.read_control(Request::SetTxvgaGain, 0, gain)?;
            if buf[0] == 0 {
                panic!("What is this return value?")
            } else {
                Ok(())
            }
        }
    }

    /// Antenna power port control. Dhruv's guess: is this DC bias?
    ///
    /// The source docs are a little lacking in terms of explanations here.
    pub fn set_antenna_enable(&self, value: bool) -> Result<()> {
        let value = if value { 1 } else { 0 };
        self.write_control(Request::AntennaEnable, value, 0, &[])
    }
}

// Helper for set_freq
fn freq_params(hz: u64) -> [u8; 8] {
    const MHZ: u64 = 1_000_000;

    let l_freq_mhz: u32 = u32::try_from(hz / MHZ).unwrap_or(u32::MAX).to_le();
    let l_freq_hz: u32 = u32::try_from(hz - u64::from(l_freq_mhz) * MHZ)
        .unwrap_or(u32::MAX)
        .to_le();

    [
        (l_freq_mhz & 0xFF) as u8,
        ((l_freq_mhz >> 8) & 0xFF) as u8,
        ((l_freq_mhz >> 16) & 0xFF) as u8,
        ((l_freq_mhz >> 24) & 0xFF) as u8,
        (l_freq_hz & 0xFF) as u8,
        ((l_freq_hz >> 8) & 0xFF) as u8,
        ((l_freq_hz >> 16) & 0xFF) as u8,
        ((l_freq_hz >> 24) & 0xFF) as u8,
    ]
}

#[cfg(test)]
mod test {
    use std::time::Duration;

    use super::*;

    #[test]
    fn test_freq_params() {
        assert_eq!(freq_params(915_000_000), [0x93, 0x03, 0, 0, 0, 0, 0, 0]);
        assert_eq!(freq_params(915_000_001), [0x93, 0x03, 0, 0, 1, 0, 0, 0]);
        assert_eq!(
            freq_params(123456789),
            [0x7B, 0, 0, 0, 0x55, 0xF8, 0x06, 0x00]
        );

        assert_eq!(freq_params(0), [0; 8]);

        assert_eq!(freq_params(u64::MAX), [0xFF; 8]);
    }

    // NOTE: make sure you can transmit on the frequency below and that you have the correct
    // antenna / attenuation before enabling!
    // #[test]
    #[allow(dead_code)]
    fn device_states() {
        let radio = HackRf::open_first().expect("Failed to open hackrf");

        radio
            .start_tx(&Config {
                vga_db: 0,
                txvga_db: 0,
                lna_db: 0,
                amp_enable: false,
                antenna_enable: false,
                frequency_hz: 915_000_000,
                sample_rate_hz: 2_000_000,
                sample_rate_div: 1,
            })
            .unwrap();
        std::thread::sleep(Duration::from_millis(50));

        radio.stop_tx().unwrap();
        assert!(radio.stop_tx().is_err());
        assert!(radio.stop_tx().is_err());
        assert!(radio.stop_rx().is_err());
        assert!(radio.stop_rx().is_err());

        std::thread::sleep(Duration::from_millis(50));

        radio
            .start_rx(&Config {
                vga_db: 0,
                txvga_db: 0,
                lna_db: 0,
                amp_enable: false,
                antenna_enable: false,
                frequency_hz: 915_000_000,
                sample_rate_hz: 2_000_000,
                sample_rate_div: 1,
            })
            .unwrap();
        std::thread::sleep(Duration::from_millis(50));

        radio.stop_rx().unwrap();
        assert!(radio.stop_rx().is_err());
        assert!(radio.stop_rx().is_err());
        assert!(radio.stop_tx().is_err());
        assert!(radio.stop_tx().is_err());
    }
}
