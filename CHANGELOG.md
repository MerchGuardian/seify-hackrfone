# ChangeLog

This format is based on [Keep a Changelog](https://keepachangelog.com/)
and this project adheres to [Semantic Versioning](https://semver.org).


## [0.2.0] - 2025-10-10
## Added
* Tx streamer support

## [0.1.1] - 2024-10-10
## Added
* Bugfixes for seify

## [0.1.0] - 2021-06-28
## Added
* Support for opening devices, scanning the USB bus
* Reading hardware version and firmware version
* Starting and stopping tx
  * Supports the following transceiver parameters: rx vga gain, tx vga gain, lna, amplifier enable, antenna power enable, frequency, sample rate
  * Changing transceiver mode (start tx, start rx, stop)
  * Reading and writing samples in a std::io::[Read, Write] fashon
  * Multi buffer reading (Using nusb::Queue)

