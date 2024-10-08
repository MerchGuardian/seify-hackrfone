use anyhow::{Context, Result};
use seify_hackrfone::{Config, HackRf};
use std::time::{Duration, Instant};

fn main() -> Result<()> {
    env_logger::init();

    let radio = HackRf::open_first().context("Failed to open Hackrf")?;

    println!("Board ID: {}", radio.board_id().context("Read board id")?);
    println!(
        "Firmware version: {}",
        radio.version().context("Read board version")?
    );
    println!("Device version: {}", radio.device_version());

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
        .context("Failed to start rx")?;

    // Starts to drop samples below 8KB MTU, with 20MHz sample rate, on AMD Ryzen 9 @5.3GHz, linux 6.6
    // Or use `start_rx_stream` so that triple buffering is used, and the kernel can continue to do
    // IO in the background while we do our processing. Expect to need rx stream or a larger MTU as
    // you implement processing more complicated than average power
    const MTU: usize = 32 * 1024;
    let mut buf = vec![0u8; MTU];

    let mut last_print = Instant::now();
    let mut bytes = 0;
    let mut stream_power = 0;
    loop {
        let n = radio.read(&mut buf).context("Failed to receive samples")?;
        assert_eq!(n, buf.len());
        bytes += n;
        for value in buf.iter().map(|v| *v as i8) {
            stream_power += (value as i64 * value as i64) as u64;
        }

        let now = Instant::now();
        let elapsed = now.saturating_duration_since(last_print);
        if elapsed.as_secs() >= 1 {
            let full_scale_ratio = stream_power as f64 / (bytes * 127 * 127) as f64;
            let db_full_scale = 10.0 * full_scale_ratio.log10() + 3.0;

            let mib = bytes as f64 / 1_000_000.0 / elapsed.as_secs_f64();
            println!(
                "{mib:.1}MiB / s, {:.1}M samples, average power {db_full_scale:.1}dBfs",
                bytes as f64 / 2.0 / 1_000_000.0,
            );
            last_print += Duration::from_secs(1);
            stream_power = 0;
            bytes = 0;
        }
    }
}
