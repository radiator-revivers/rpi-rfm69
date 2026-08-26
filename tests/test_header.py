"""Golden-vector + round-trip tests for the pure 10-bit header codec.

The golden vectors are asserted as literal wire bytes (AC-2) — not recomputed
from the packer's own expression — so the code cannot tautologically satisfy
them. The round-trip test (AC-3) sweeps the full ID range and flag combinations.
"""
import itertools

import pytest

from RFM69.header import pack_header, parse_header


def test_conftest_smoke():
    """The forked package imports host-side under the conftest stubs (AC-1)."""
    import RFM69  # noqa: F401  pylint: disable=import-outside-toplevel
    assert RFM69 is not None


# AC-2: (args, kwargs) -> exact 3 wire bytes as a hex string.
GOLDEN = [
    ((1, 1), {}, "010100"),
    ((1, 900), {}, "018403"),
    ((1023, 1), {}, "ff010c"),
    ((1023, 900), {"request_ack": True}, "ff844f"),
    ((1, 1), {"send_ack": True}, "010180"),
    ((1, 1), {"send_ack": True, "rssi_request": True}, "0101a0"),
]


@pytest.mark.parametrize("args,kwargs,expected_hex", GOLDEN)
def test_pack_header_golden(args, kwargs, expected_hex):
    assert pack_header(*args, **kwargs).hex() == expected_hex


IDS = [0, 1, 255, 256, 900, 1023]
# (request_ack, send_ack, rssi_request) — send_ack wins over request_ack in CTL.
FLAG_COMBOS = list(itertools.product([False, True], repeat=3))


@pytest.mark.parametrize("target", IDS)
@pytest.mark.parametrize("sender", IDS)
@pytest.mark.parametrize("request_ack,send_ack,rssi_request", FLAG_COMBOS)
def test_round_trip(target, sender, request_ack, send_ack, rssi_request):
    """parse_header(pack_header(...)) reconstructs ids + flags (AC-3)."""
    parsed = parse_header(
        pack_header(
            target, sender,
            request_ack=request_ack, send_ack=send_ack, rssi_request=rssi_request,
        )
    )
    assert parsed.target == target
    assert parsed.sender == sender
    assert parsed.rssi_requested == rssi_request
    # send_ack takes the 0x80 slot and preempts the 0x40 request_ack bit.
    assert parsed.ack_received == send_ack
    assert parsed.ack_requested == (request_ack and not send_ack)


@pytest.mark.parametrize("bad", [1024, -1, 2000])
def test_out_of_range_target(bad):
    with pytest.raises(ValueError):
        pack_header(bad, 1)


@pytest.mark.parametrize("bad", [1024, -1, 2000])
def test_out_of_range_sender(bad):
    with pytest.raises(ValueError):
        pack_header(1, bad)
