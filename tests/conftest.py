"""Host-side import shim.

The forked ``RFM69`` package imports ``spidev`` and ``RPi.GPIO`` at module load
time (via ``RFM69/__init__`` -> ``radio``). Those libraries only exist on a
Raspberry Pi, so on a development Mac ``import RFM69`` would fail before any test
could run. conftest is imported by pytest before test collection, so inserting
fake modules here covers every test in ``tests/``.
"""
import sys
import types
from unittest.mock import MagicMock

for _name in ("spidev", "RPi", "RPi.GPIO"):
    if _name not in sys.modules:
        _mod = types.ModuleType(_name)
        # Any attribute access (classes, constants, functions) returns a Mock.
        _mod.__getattr__ = lambda _attr: MagicMock()  # type: ignore[attr-defined]
        sys.modules[_name] = _mod

# Make ``RPi.GPIO`` reachable as an attribute of ``RPi`` (import machinery expects it).
sys.modules["RPi"].GPIO = sys.modules["RPi.GPIO"]

# --- Mocked-SPI Radio harness (phase 02) --------------------------------------
# The upstream tests/test_radio*.py build a real ``Radio`` and busy-wait on
# hardware, so they only run on-Pi. To exercise ``_sendFrame`` / ``_interruptHandler``
# host-side we build a ``Radio`` via ``__new__`` + attribute injection (no ``__init__``,
# no SPI/GPIO) and drive it through a small ``FakeSpiDev`` double.
import threading  # noqa: E402  pylint: disable=wrong-import-position

import pytest  # noqa: E402  pylint: disable=wrong-import-position

from RFM69.radio import Radio  # noqa: E402  pylint: disable=wrong-import-position
from RFM69.header import pack_header  # noqa: E402  pylint: disable=wrong-import-position
from RFM69.registers import REG_FIFO, RF69_MODE_RX  # noqa: E402  pylint: disable=wrong-import-position


class FakeSpiDev:
    """Minimal SpiDev test double.

    - Records every write (address byte has bit 7 set) in ``writes`` for TX
      assertions.
    - Serves FIFO reads (address byte == ``REG_FIFO & 0x7f`` == 0) by popping
      successive bytes from a primed ``rx_fifo`` stream — mirroring how the real
      chip pops the FIFO on each burst read.
    - Returns a benign ``0xFF`` for every other register read, so the driver's
      ``MODEREADY`` / ``PAYLOADREADY`` / ``PACKETSENT`` mask-checks fall through
      immediately (never busy-waits) and ``REG_RSSIVALUE`` yields a real int.
    """

    def __init__(self):
        self.writes = []
        self.rx_fifo = b""
        self._cursor = 0
        # Per-register read overrides (keyed by the 7-bit register address).
        # Read-based accessors (bitrate, fdev, version, CRC/AES/sync flags) need
        # *specific* register values; anything not primed still reads 0xFF so the
        # MODEREADY / PAYLOADREADY / PACKETSENT mask-checks keep falling through.
        self.reg_values = {}

    def prime(self, data):
        self.rx_fifo = bytes(data)
        self._cursor = 0

    def _xfer(self, data):
        first = data[0]
        if first & 0x80:                        # register / FIFO write
            self.writes.append(list(data))
            return [0] * len(data)
        if first == (REG_FIFO & 0x7f):          # FIFO read: pop from the stream
            n = len(data) - 1
            chunk = self.rx_fifo[self._cursor:self._cursor + n]
            self._cursor += n
            return [0] + list(chunk) + [0] * (n - len(chunk))
        # plain register read: serve a primed value if we have one, else 0xFF
        return [0] + [self.reg_values.get(first & 0x7f, 0xFF)] * (len(data) - 1)

    xfer = _xfer
    xfer2 = _xfer


@pytest.fixture
def radio():
    """A hardware-free ``Radio``: ``__new__`` + only the attributes that
    ``_sendFrame`` / ``_interruptHandler`` (and the helpers they call) touch."""
    r = Radio.__new__(Radio)
    r.spi = FakeSpiDev()
    r.address = 300                 # 10-bit id -> exercises the high-bit path
    r.isRFM69HW = False
    r.promiscuousMode = 0
    r.enableATC = False
    r.enableRSSIack = False
    r.auto_acknowledge = True
    # ATC auto-power members (phase 04d). The fixture bypasses __init__, so these
    # must be primed by hand. powerLevel is set by direct attribute (NOT via
    # set_power_level_raw) so it records no REG_PALEVEL write and the "no write"
    # convergence assertions hold.
    r._targetRSSI = 0
    r._ackRSSI = 0
    r._transmitLevelStep = 1
    r.powerLevel = 31
    r.lastRSSI = 0
    r.logger = None
    r._packets = []
    r.acks = {}
    r.mode = RF69_MODE_RX
    r._spiLock = threading.Lock()
    r._intLock = threading.Lock()
    r._sendLock = threading.Condition()
    r._ackLock = threading.Condition()
    r._packetLock = threading.Condition()
    r._modeLock = threading.RLock()
    return r


@pytest.fixture
def prime():
    """Return a helper that stages an RX frame in the radio's fake FIFO."""
    def _prime(radio, target, sender, payload, **flags):  # pylint: disable=redefined-outer-name
        radio.spi.prime(
            bytes([len(payload) + 3])
            + pack_header(target, sender, **flags)
            + bytes(payload)
        )
    return _prime
