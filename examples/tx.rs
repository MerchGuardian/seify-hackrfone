use std::sync::Arc;

use anyhow::{Context, Result};
use seify_hackrfone::{Config, HackRf};

fn main() -> Result<()> {
    let mut builder = env_logger::builder();
    builder.filter_level(log::LevelFilter::Info);
    builder.init();

    let radio = Arc::new(HackRf::open_first().context("Failed to open Hackrf")?);

    println!("Board ID: {}", radio.board_id().context("Read board id")?);
    println!(
        "Firmware version: {}",
        radio.version().context("Read board version")?
    );
    println!("Device version: {}", radio.device_version());

    for _ in 0..10 {
        radio
            .start_tx(&Config {
                vga_db: 0,
                txvga_db: 32,
                lna_db: 0,
                amp_enable: false,
                antenna_enable: false,
                frequency_hz: 917_500_000,
                sample_rate_hz: 2_000_000,
                sample_rate_div: 1,
            })
            .context("Failed to start rx")?;

        const MTU: usize = 32 * 1024;
        let buf = vec![0u8; MTU];
        let mut tx = radio.start_tx_stream()?;

        for _ in 0..100 {
            tx.write_sync(&buf).context("Failed to write samples")?;
        }
    }

    Ok(())
}
