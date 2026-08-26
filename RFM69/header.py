"""Pure 10-bit-addressing header codec (no hardware dependencies).

Mirrors LowPowerLab's 10-bit header as used by the production Moteino gateway
(``forked_RFM69.cpp``). Node IDs span 0..1023; the two high bits of each ID are
smuggled into the control byte:

  wire bytes = [ target & 0xFF, sender & 0xFF, CTL ]

  CTL bits:
    0x80  ACK sent      (send_ack)
    0x40  ACK requested (request_ack)
    0x20  RSSI request  (RFM69_CTL_RESERVE1, ATC reactive-RSSI echo)
    0x0C  target high bits  (target & 0x300) >> 6
    0x03  sender high bits  (sender & 0x300) >> 8

This module imports no ``spidev``/``RPi.GPIO`` so it is unit-testable on any host.
"""
from typing import NamedTuple


class ParsedHeader(NamedTuple):
    """Decoded 10-bit header."""
    target: int
    sender: int
    ack_received: bool
    ack_requested: bool
    rssi_requested: bool


def _check_id(name, value):
    if not 0 <= value <= 1023:
        raise ValueError(f"{name} must be in 0..1023, got {value}")


def pack_header(target, sender, *, request_ack=False, send_ack=False,
                rssi_request=False):
    """Pack a 3-byte 10-bit header.

    ``send_ack`` and ``request_ack`` are mutually exclusive in the CTL byte
    (send_ack wins), matching ``forked_RFM69.cpp`` ``sendFrame``.
    """
    _check_id("target", target)
    _check_id("sender", sender)

    if send_ack:
        ctl = 0x80
    elif request_ack:
        ctl = 0x40
    else:
        ctl = 0x00

    if rssi_request:
        ctl |= 0x20

    if target > 0xFF:
        ctl |= (target & 0x300) >> 6
    if sender > 0xFF:
        ctl |= (sender & 0x300) >> 8

    return bytes([target & 0xFF, sender & 0xFF, ctl])


def parse_header(data):
    """Parse a 3-byte 10-bit header into a :class:`ParsedHeader`."""
    ctl = data[2]
    return ParsedHeader(
        target=data[0] | ((ctl & 0x0C) << 6),
        sender=data[1] | ((ctl & 0x03) << 8),
        ack_received=bool(ctl & 0x80),
        ack_requested=bool(ctl & 0x40),
        rssi_requested=bool(ctl & 0x20),
    )
