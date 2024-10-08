use anyhow::{Context, Result};
use seify_hackrfone::HackRf;

fn main() -> Result<()> {
    env_logger::init();

    let radio = HackRf::open_first().context("Failed to open Hackrf")?;

    println!("Board ID: {}", radio.board_id().context("Read board id")?);
    println!(
        "Firmware version: {}",
        radio.version().context("Read board version")?
    );
    println!("Device version: {}", radio.device_version());

    Ok(())
}
