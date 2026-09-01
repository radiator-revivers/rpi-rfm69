# Changelog

## 1.6.0 (in progress)
Parity pass toward LowPowerLab RFM69 v1.6.0, as the standalone fork
`rpi-rfm69-rr`. Host-provable additions land first (04c); ATC auto-power (04d)
and Listen-Mode RX (04e) follow on hardware.

### Added (04c - API accessor parity)
- Power API: `set_power_level_raw(level)` (raw 0-31 port of upstream
  `setPowerLevel`), `set_power_dbm(dbm)` (`setPowerDBm`), `get_power_level()`.
  `set_power_level(percent)` now delegates to the raw path; its percent-in /
  `REG_PALEVEL`-out contract is byte-identical.
- `set_lna(new_reg)` (`setLNA`); `_reinitRadio` now routes its REG_LNA write
  through it.
- `read_registers_compact()` - structured `{addr: value}` dump (`readAllRegsCompact`).
- `spy_mode(on_off=True)` / `get_spy_mode()` over the existing `promiscuousMode`.
- Accessor cluster: `get_version`, `get_address`, `get_network`, `get_bitrate`,
  `get_frequency_deviation`, `is_crc_on`, `is_aes_on`, `is_sync_on`,
  `is_high_power`, `get_output_power`, `dbm_to_mw`.

### Added (04d - ATC closed-loop auto-power, TX side)
- ATC auto-power sender half (port of `RFM69_ATC`): `enable_auto_power(target_rssi)`,
  `get_ack_rssi()`, `get_target_rssi()`. An ack-requested send now sets the RSSI-echo
  request bit (CTL `0x20`) whenever a target is set, and transmit power converges
  toward the target from the RSSI the receiver echoes back in the ACK. Independent of
  `enableATC` (the responder echo shipped in phase 02); a fully-ATC node runs both.

### Changed
- `_setAddress` masks `REG_NODEADRS` to 8 bits so a 10-bit node id never
  overflows the SPI byte; `self.address` still holds the full 10-bit id.

### Intentionally omitted
- `setISRCallback` - the port owns its IRQ via
  `GPIO.add_event_detect(_interruptHandler)`; a user ISR hook does not fit the
  model, so it is deliberately not ported.

## 0.7.0
- RFM69HW and HCW specific functions and power level setting added by @Makodan

## 0.6.0
- Added support for ATC mode (thanks @MxMarx)
- Reduced some hang (thanks @MxMarx)
- Extended registers retrieved to include High Power PA settings (thanks @tomtastic)

## 0.5.1
- Added support for radios without reset pins

## 0.5.0
- Added set_frequency_in_Hz and get_frequency_in_Hz

## 0.4.0
- Made the Radio class threadsafe, and added threadsafe methods for accessing packets
- Added testing for the threadsafe methods
- Added pylinting and made some cosmetic changes to get a good pylint score
- Added coverage testing via coveralls.io, and instructions for doing so

## 0.3.0
- Added support for sendListenModeBurst
- Made tests more configurable
- Removed Python 2 from tests since it's EOL
- Added instructions on how to build for PyPi
