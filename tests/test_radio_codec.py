"""Wire-level tests for the 10-bit codec integrated into ``radio.py``.

Driven through a mocked-SPI ``Radio`` (see ``conftest.FakeSpiDev`` + the ``radio``
and ``prime`` fixtures) so ``_sendFrame`` and ``_interruptHandler`` run host-side
on the Mac — no bonnet. The TX assertion pins the *literal* on-air header bytes so
a regression to the old 8-bit masking (dropping the high address bits) fails loudly.
"""
# pylint: disable=protected-access,missing-function-docstring,redefined-outer-name
from unittest.mock import MagicMock

from RFM69.packet import Packet
from RFM69.registers import REG_FIFO


def _fifo_write(spi):
    """The single FIFO burst-write (address byte == ``REG_FIFO | 0x80`` == 0x80)."""
    for w in spi.writes:
        if w[0] == (REG_FIFO | 0x80):
            return w
    raise AssertionError("no FIFO burst-write was recorded")


# AC-1 -------------------------------------------------------------------------
def test_sendframe_packs_10bit_header(radio):
    # address 900 -> target 1023, requestACK: header bytes must be ff 84 4f
    radio.address = 900
    radio._sendFrame(1023, [1, 2, 3], True, False)
    assert _fifo_write(radio.spi) == [REG_FIFO | 0x80, 6, 0xFF, 0x84, 0x4F, 1, 2, 3]


# AC-2 -------------------------------------------------------------------------
def test_interrupt_recovers_10bit_and_ack_requested(radio, prime):
    radio.send_ack = MagicMock()            # isolate the packet from the auto-ack path
    prime(radio, target=300, sender=900, payload=[10, 20], request_ack=True)
    radio._interruptHandler(0)

    assert len(radio._packets) == 1
    pkt = radio._packets[0]
    assert pkt.receiver == 300              # this node (10-bit)
    assert pkt.sender == 900                # full 10-bit sender recovered from CTL
    assert pkt.ack_requested is True
    assert pkt.data == [10, 20]


# AC-3 -------------------------------------------------------------------------
def test_rssi_echo_is_reactive(radio, prime):
    # RSSI-request bit set on the incoming frame -> RSSI-payload ack, even though
    # the global enableRSSIack override is OFF (reactive echo off CTL 0x20).
    radio.enableRSSIack = False
    radio.send_ack = MagicMock()
    prime(radio, target=300, sender=900, payload=[1], request_ack=True, rssi_request=True)
    radio._interruptHandler(0)

    radio.send_ack.assert_called_once()
    args, _ = radio.send_ack.call_args
    assert args[0] == 900
    assert len(args[1]) == 1                # 1-byte abs(RSSI) echo payload


def test_normal_ack_when_no_rssi_bit(radio, prime):
    # Same frame without the RSSI bit -> plain ack, no payload arg.
    radio.enableRSSIack = False
    radio.send_ack = MagicMock()
    prime(radio, target=300, sender=900, payload=[1], request_ack=True)
    radio._interruptHandler(0)

    radio.send_ack.assert_called_once()
    args, _ = radio.send_ack.call_args
    assert args[0] == 900
    assert len(args) == 1 or len(args[1]) == 0   # send_ack(sender) with no payload


# AC-4 -------------------------------------------------------------------------
def test_packet_ack_requested_field():
    p = Packet(1, 2, -30, [9], ack_requested=True)
    assert p.ack_requested is True
    assert p.to_dict()["ack_requested"] is True

    d = Packet(1, 2, -30, [9])
    assert d.ack_requested is False
    assert d.to_dict()["ack_requested"] is False
