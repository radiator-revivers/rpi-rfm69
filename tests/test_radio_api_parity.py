"""Host-side parity tests for the LowPowerLab RFM69 v1.6.0 API surface (phase 04c).

Driven through the mocked-SPI ``Radio`` (see ``conftest.FakeSpiDev`` + the
``radio`` fixture). Read-based accessors are exercised by priming
``radio.spi.reg_values`` with specific register bytes; every assertion is an
analytic golden vector computed from the v1.6.0 register math - the same oracle
discipline as ``test_radio_codec.py``. No hardware is touched.
"""
# pylint: disable=protected-access,missing-function-docstring,redefined-outer-name
import pytest

from RFM69.registers import (
    REG_FIFO,
    REG_PALEVEL, REG_LNA, REG_NODEADRS, REG_VERSION,
    REG_BITRATEMSB, REG_BITRATELSB, REG_FDEVMSB, REG_FDEVLSB,
    REG_PACKETCONFIG1, REG_PACKETCONFIG2, REG_SYNCCONFIG,
    RF_PALEVEL_PA0_ON, RF_PALEVEL_PA1_ON, RF_PALEVEL_PA2_ON,
    RF_PACKET1_CRC_ON, RF_PACKET2_AES_ON, RF_SYNC_ON,
    FXOSC, FSTEP,
)


def _last_write(spi, reg):
    """Value byte of the last ``_writeReg(reg, ...)`` recorded, or None."""
    for w in reversed(spi.writes):
        if w[0] == (reg | 0x80):
            return w[1]
    return None


def _palevel(pa_setting, level):
    """Recompute the expected REG_PALEVEL byte from the PA setting + adjusted level."""
    return pa_setting | level


# --- T2: power API parity -----------------------------------------------------
# Golden vectors for the HW/HCW dBm -> REG_PALEVEL path (set_power_dbm delegates
# to set_power_level_raw, which applies the PA1 / PA1+PA2 / +HiPower mapping).
HW_DBM_VECTORS = [
    # dbm, expected REG_PALEVEL byte, expected stored powerLevel
    (-5, _palevel(RF_PALEVEL_PA1_ON, 0 + 16), 0),                 # clamps to -2 -> raw 0
    (-2, _palevel(RF_PALEVEL_PA1_ON, 0 + 16), 0),                 # raw 2+(-2)=0
    (0,  _palevel(RF_PALEVEL_PA1_ON, 2 + 16), 2),                 # raw 2
    (13, _palevel(RF_PALEVEL_PA1_ON, 15 + 16), 15),              # raw 15 (< 16)
    (14, _palevel(RF_PALEVEL_PA1_ON | RF_PALEVEL_PA2_ON, 16 + 10), 16),   # raw 16 (< 20)
    (17, _palevel(RF_PALEVEL_PA1_ON | RF_PALEVEL_PA2_ON, 20 + 8), 20),    # raw 3+17=20 (>=20)
    (20, _palevel(RF_PALEVEL_PA1_ON | RF_PALEVEL_PA2_ON, 23 + 8), 23),    # raw 23
    (25, _palevel(RF_PALEVEL_PA1_ON | RF_PALEVEL_PA2_ON, 23 + 8), 23),    # clamps to 20 -> raw 23
]


@pytest.mark.parametrize("dbm,expected_palevel,expected_level", HW_DBM_VECTORS)
def test_set_power_dbm_hw_writes_exact_palevel(radio, dbm, expected_palevel, expected_level):
    radio.isRFM69HW = True
    returned = radio.set_power_dbm(dbm)
    assert _last_write(radio.spi, REG_PALEVEL) == expected_palevel
    assert radio.powerLevel == expected_level
    assert radio.get_power_level() == expected_level
    # HW path returns the (clamped) dbm actually applied.
    assert returned == max(-2, min(20, dbm))


@pytest.mark.parametrize("dbm,expected", [(5, 5), (13, 13), (20, 13), (-30, -18)])
def test_set_power_dbm_wcw_only_clamps_no_write(radio, dbm, expected):
    # W/CW: upstream setPowerDBm only clamps and returns; it writes NOTHING.
    radio.isRFM69HW = False
    assert radio.set_power_dbm(dbm) == expected
    assert _last_write(radio.spi, REG_PALEVEL) is None


def test_set_power_level_percent_unchanged_wcw(radio):
    # Regression pin: set_power_level(70) on a W/CW must write the SAME byte as
    # before the raw-path refactor. round(31*0.7)=22, PA0_ON|22.
    radio.isRFM69HW = False
    radio.set_power_level(70)
    assert _last_write(radio.spi, REG_PALEVEL) == (RF_PALEVEL_PA0_ON | 22)
    assert radio.powerLevel == 22


def test_set_power_level_percent_unchanged_hw(radio):
    # Regression pin (HW): set_power_level(100) -> round(31)=31, clamp to 23,
    # +8 -> 31, PA1+PA2.
    radio.isRFM69HW = True
    radio.set_power_level(100)
    assert _last_write(radio.spi, REG_PALEVEL) == (RF_PALEVEL_PA1_ON | RF_PALEVEL_PA2_ON | 31)
    assert radio.powerLevel == 23


def test_set_power_level_raw_takes_raw_not_percent(radio):
    # A raw level of 2 must land at PA1_ON|(2+16), NOT be treated as a percent.
    radio.isRFM69HW = True
    radio.set_power_level_raw(2)
    assert _last_write(radio.spi, REG_PALEVEL) == (RF_PALEVEL_PA1_ON | 18)
    assert radio.powerLevel == 2


# --- T3: set_lna --------------------------------------------------------------
def test_set_lna_masks_low_three_bits_and_returns_old(radio):
    radio.spi.reg_values[REG_LNA] = 0x8F           # 1000 1111
    old = radio.set_lna(0x02)                        # new gain bits = 010
    # keep old high bits (0x88), replace low 3 with 2 -> 1000 1010
    assert _last_write(radio.spi, REG_LNA) == ((0x02 & 7) | (0x8F & ~7))
    assert _last_write(radio.spi, REG_LNA) == 0x8A
    assert old == 0x8F


# --- T4: read_registers_compact ----------------------------------------------
def test_read_registers_compact_maps_primed_values(radio):
    radio.spi.reg_values[REG_VERSION] = 0x24
    radio.spi.reg_values[0x20] = 0x42
    dump = radio.read_registers_compact()
    assert dump[REG_VERSION] == 0x24
    assert dump[0x20] == 0x42
    assert 0x00 not in dump                          # starts at 0x01
    assert min(dump) == 0x01 and max(dump) == 0x7F
    assert len(dump) == 0x7F


# --- T5: spy_mode / get_spy_mode ---------------------------------------------
def test_spy_mode_aliases_promiscuous_both_ways(radio):
    radio.spy_mode(True)
    assert radio.get_spy_mode() is True
    assert radio.promiscuousMode is True
    radio.spy_mode(False)
    assert radio.get_spy_mode() is False
    assert radio.promiscuousMode is False


# --- T6: accessor cluster (golden vectors from v1.6.0 register math) ----------
def test_get_version(radio):
    radio.spi.reg_values[REG_VERSION] = 0x24
    assert radio.get_version() == 0x24


def test_get_address_and_network(radio):
    radio.address = 500
    radio._networkID = 100
    assert radio.get_address() == 500
    assert radio.get_network() == 100


def test_get_bitrate(radio):
    # 0x0240 = 576 -> 32_000_000 // 576 == 55555 bps
    radio.spi.reg_values[REG_BITRATEMSB] = 0x02
    radio.spi.reg_values[REG_BITRATELSB] = 0x40
    assert radio.get_bitrate() == FXOSC // 576
    assert radio.get_bitrate() == 55555


def test_get_frequency_deviation(radio):
    # 0x0666 = 1638 -> int(61.03515625 * 1638) == 99975 Hz
    radio.spi.reg_values[REG_FDEVMSB] = 0x06
    radio.spi.reg_values[REG_FDEVLSB] = 0x66
    assert radio.get_frequency_deviation() == int(FSTEP * 1638)
    assert radio.get_frequency_deviation() == 99975


@pytest.mark.parametrize("val,expected", [(RF_PACKET1_CRC_ON, True), (0x00, False)])
def test_is_crc_on(radio, val, expected):
    radio.spi.reg_values[REG_PACKETCONFIG1] = val
    assert radio.is_crc_on() is expected


@pytest.mark.parametrize("val,expected", [(RF_PACKET2_AES_ON, True), (0x00, False)])
def test_is_aes_on(radio, val, expected):
    radio.spi.reg_values[REG_PACKETCONFIG2] = val
    assert radio.is_aes_on() is expected


@pytest.mark.parametrize("val,expected", [(RF_SYNC_ON, True), (0x00, False)])
def test_is_sync_on(radio, val, expected):
    radio.spi.reg_values[REG_SYNCCONFIG] = val
    assert radio.is_sync_on() is expected


@pytest.mark.parametrize("hw", [True, False])
def test_is_high_power(radio, hw):
    radio.isRFM69HW = hw
    assert radio.is_high_power() is hw


def test_get_output_power(radio):
    radio.spi.reg_values[REG_PALEVEL] = 0x9F         # PA0_ON | 0x1F
    assert radio.get_output_power() == 0x1F


@pytest.mark.parametrize("dbm,expected", [(0, 1.0), (10, 10.0), (20, 100.0), (-10, 0.1)])
def test_dbm_to_mw(radio, dbm, expected):
    assert radio.dbm_to_mw(dbm) == pytest.approx(expected)


# --- T7: _setAddress 10-bit hardening ----------------------------------------
def test_setaddress_masks_reg_nodeadrs_but_keeps_full_address(radio):
    radio._setAddress(500)
    assert _last_write(radio.spi, REG_NODEADRS) == (500 & 0xFF)   # 0xF4
    assert radio.address == 500


# --- 04d: ATC closed-loop auto-power (TX side) --------------------------------
# CTL bits (header.py): 0x40 REQACK, 0x20 RSSI-request (RFM69_CTL_RESERVE1).
CTL_REQACK = 0x40
CTL_RSSI_REQUEST = 0x20


def _fifo_ctl(spi):
    """CTL byte (index 4) of the recorded FIFO burst-write.

    Layout: ``[REG_FIFO|0x80, len+3, target, sender, CTL, ...payload]``.
    """
    for w in spi.writes:
        if w[0] == (REG_FIFO | 0x80):
            return w[4]
    raise AssertionError("no FIFO burst-write was recorded")


def test_enable_auto_power_sets_target_and_ack_rssi_gating(radio):
    # get_ack_rssi is 0 while auto-power is off, even if _ackRSSI holds a value.
    radio._ackRSSI = -55
    assert radio.get_ack_rssi() == 0
    radio.enable_auto_power(-80)
    assert radio.get_target_rssi() == -80
    # With a target set, get_ack_rssi returns the captured echo.
    radio._update_ack_rssi_and_power(75)             # abs(rssi)=75 -> -75 dBm
    assert radio.get_ack_rssi() == -75


def test_default_target_rssi_is_zero_before_enable(radio):
    assert radio.get_target_rssi() == 0
    assert radio.get_ack_rssi() == 0


# AC-2: drive the REAL send path and read the recorded CTL byte (a pack_header-
# only assertion would pass without the radio.py _sendFrame edit).
def test_ack_requested_send_sets_rssi_request_bit_when_auto_power_on(radio):
    radio.enable_auto_power(-70)
    radio._sendFrame(100, [1, 2, 3], True, False)    # requestACK=True, sendACK=False
    ctl = _fifo_ctl(radio.spi)
    assert ctl & CTL_REQACK == CTL_REQACK            # still an ack request
    assert ctl & CTL_RSSI_REQUEST == CTL_RSSI_REQUEST  # + RSSI echo requested


def test_ack_requested_send_leaves_rssi_bit_clear_when_auto_power_off(radio):
    # Regression pin: _targetRSSI == 0 -> requestACK frame byte-identical to today.
    assert radio._targetRSSI == 0
    radio._sendFrame(100, [1, 2, 3], True, False)
    ctl = _fifo_ctl(radio.spi)
    assert ctl & CTL_REQACK == CTL_REQACK
    assert ctl & CTL_RSSI_REQUEST == 0


# AC-3: convergence table driven through the extracted decision method.
# (isRFM69HW, powerLevel, target, rssi_byte, exp_level, exp_palevel, exp_ack_rssi)
CONVERGENCE_VECTORS = [
    # weak echo below target, below max -> +step (W/CW: REG_PALEVEL = PA0_ON|level)
    (False, 10, -70, 90, 11, RF_PALEVEL_PA0_ON | 11, -90),
    # strong echo above target, above 0 -> -1
    (False, 10, -70, 50, 9, RF_PALEVEL_PA0_ON | 9, -50),
    # at W/CW max (31), weak echo -> no increase, no write
    (False, 31, -70, 90, 31, None, -90),
    # at HW max (23), weak echo -> no increase, no write
    (True, 23, -70, 90, 23, None, -90),
    # at 0, strong echo -> no decrease, no write
    (False, 0, -70, 50, 0, None, -50),
    # exact match -> no change, no write
    (False, 10, -70, 70, 10, None, -70),
    # HW weak echo below max -> +step (HW: level 11 < 16 -> +16, PA1_ON)
    (True, 10, -70, 90, 11, RF_PALEVEL_PA1_ON | (11 + 16), -90),
]


@pytest.mark.parametrize(
    "is_hw,power,target,rssi_byte,exp_level,exp_palevel,exp_ack", CONVERGENCE_VECTORS)
def test_update_ack_rssi_and_power_steps_like_upstream(
        radio, is_hw, power, target, rssi_byte, exp_level, exp_palevel, exp_ack):
    radio.isRFM69HW = is_hw
    radio.powerLevel = power                          # direct: no spurious write
    radio._targetRSSI = target
    radio._update_ack_rssi_and_power(rssi_byte)
    assert radio.powerLevel == exp_level
    assert radio.get_power_level() == exp_level
    assert _last_write(radio.spi, REG_PALEVEL) == exp_palevel
    assert radio.get_ack_rssi() == exp_ack


def test_update_ack_rssi_and_power_target_zero_captures_but_no_step(radio):
    # target == 0: _ackRSSI is captured internally but no step, no write, and the
    # accessor still reports 0 (ATC off).
    radio.isRFM69HW = False
    radio.powerLevel = 10
    radio._targetRSSI = 0
    radio._update_ack_rssi_and_power(90)
    assert radio.powerLevel == 10
    assert _last_write(radio.spi, REG_PALEVEL) is None
    assert radio.get_ack_rssi() == 0
