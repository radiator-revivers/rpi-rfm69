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
