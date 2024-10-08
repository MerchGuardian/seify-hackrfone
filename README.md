# seify-hackrfone &emsp; [![Crates.io]][crates.io] [![Documentation]][docs.rs] [![Build Status]][actions] [![Made with Rust]][rust]

[Crates.io]: https://img.shields.io/crates/v/seify-hackrfone.svg
[crates.io]: https://crates.io/crates/seify-hackrfone
[Documentation]: https://docs.rs/seify-hackrfone/badge.svg
[docs.rs]: https://docs.rs/seify-hackrfone
[Build Status]: https://github.com/MerchGuardian/seify-hackrfone/actions/workflows/CI.yml/badge.svg
[actions]: https://github.com/MerchGuardian/seify-hackrfone/actions
[Made with Rust]: https://img.shields.io/badge/Made%20with-Rust-blue.svg
[rust]: https://www.rust-lang.org/


Rust [Hackrf One](https://greatscottgadgets.com/hackrf/one/) api, with zero native dependencies, powered by [nusb](https://github.com/kevinmehall/nusb)

Written as part of [seify]() and [FutureSDR](), but standalone usage is also supported

## Example
```
use anyhow::Result;
use seify_hackrfone::{Config, HackRf};

fn main() -> Result<()> {
    let radio = HackRf::open_first()?;

    radio.start_rx(&Config {
        vga_db: 0,
        txvga_db: 0,
        lna_db: 0,
        amp_enable: false,
        antenna_enable: false,
        frequency_hz: 915_000_000,
        sample_rate_hz: 2_000_000,
        sample_rate_div: 1,
    })?;

    let mut buf = vec![0u8; 32 * 1024];
    loop {
        radio.read(&mut buf)?;
        // Process samples...
    }
}
```
