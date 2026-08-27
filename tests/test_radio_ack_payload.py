"""Host-side tests for the ACK-payload retention + send/read-reply primitive.

Phase 04a. The published fork records only a presence sentinel for an incoming
radio ACK and drops the bytes the ACK carried, so a request/reply protocol like
WirelessHEX69 OTA (whose nodes answer via LowPowerLab ``sendACK``) is blind to
replies like ``FLX?OK`` / ``FLX:<seq>:OK``. These tests pin the fix:

- T1/AC-1: ``_interruptHandler`` retains the ACK payload in ``radio.acks`` and
  does NOT queue the ACK as a normal RX packet.
- T2/AC-2: ``send_and_get_reply`` returns the retained reply bytes, ``None`` on
  no-ACK, consumes the reply (no stale re-read), and leaves normal RX untouched.

Driven through the mocked-SPI ``radio`` + ``prime`` fixtures (see ``conftest``),
so ``_interruptHandler`` runs host-side on the Mac. An ACK frame is one whose
CTL 0x80 bit is set — ``pack_header(send_ack=True)`` — which ``parse_header``
surfaces as ``ack_received=True``.
"""
# pylint: disable=protected-access,missing-function-docstring,redefined-outer-name

# b"FLX?OK" — the WirelessHEX69 handshake/EOF reply, returned inside the radio ACK.
FLX_OK = [0x46, 0x4C, 0x58, 0x3F, 0x4F, 0x4B]


# T1 / AC-1 --------------------------------------------------------------------
def test_interrupt_retains_ack_payload(radio, prime):
    radio.address = 300
    prime(radio, target=radio.address, sender=42, payload=FLX_OK, send_ack=True)
    radio._interruptHandler(0)

    # Exact payload bytes retained, keyed by sender -- not a presence sentinel.
    assert radio.acks[42] == FLX_OK
    # ACK frame is NOT queued as a normal RX packet.
    assert len(radio._packets) == 0


def test_empty_ack_stores_present_key(radio, prime):
    # An empty-payload ACK still marks the key present so send()'s bool contract
    # (presence == "ack received") is preserved.
    radio.address = 300
    prime(radio, target=radio.address, sender=42, payload=[], send_ack=True)
    radio._interruptHandler(0)

    assert radio.acks[42] == []
    assert 42 in radio.acks


def test_send_bool_contract_unchanged(radio):
    # _ACKReceived keys on presence, ignoring the stored value, so send()'s
    # truthy-on-ack behaviour is unchanged by retaining the payload.
    radio.acks[42] = FLX_OK
    assert radio._ACKReceived(42) is True
    assert 42 not in radio.acks           # popped on read
    assert radio._ACKReceived(42) is False


# T2 / AC-2 --------------------------------------------------------------------
def test_send_and_get_reply_returns_payload(radio):
    # No interrupt thread exists host-side, so seed the reply the way the handler
    # would (T1 path is covered above), then drive the primitive.
    radio.acks[42] = FLX_OK
    assert radio.send_and_get_reply(42, [1, 2, 3]) == FLX_OK
    # Reply consumed.
    assert 42 not in radio.acks
    # Normal RX path untouched by the exchange.
    assert len(radio._packets) == 0


def test_send_and_get_reply_none_on_no_ack(radio):
    # Small wait/attempts so the timeout path doesn't burn the module default.
    assert radio.send_and_get_reply(42, [1, 2, 3], wait=10, attempts=1) is None


def test_send_and_get_reply_consumes_reply(radio):
    radio.acks[42] = FLX_OK
    assert radio.send_and_get_reply(42, [1, 2, 3]) == FLX_OK
    # A later call sees no stale reply.
    assert radio.send_and_get_reply(42, [1, 2, 3], wait=10, attempts=1) is None
