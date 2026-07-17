from types import SimpleNamespace

from ipe_control.reference_manager import ReferenceManager


def test_duration_seconds() -> None:
    duration = SimpleNamespace(sec=2, nanosec=500_000_000)
    assert ReferenceManager._duration_seconds(duration) == 2.5
