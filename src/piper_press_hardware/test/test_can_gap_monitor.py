import struct
import socket

from piper_press_hardware.can_gap_monitor import (
    GapTracker,
    is_joint_feedback_frame,
    monitor,
)


def test_only_standard_joint_data_frames_count():
    frame = struct.pack('=IB3x8s', 0x2A3, 8, bytes(8))
    assert is_joint_feedback_frame(frame)
    assert not is_joint_feedback_frame(struct.pack('=IB3x8s', 0x2A2, 8, bytes(8)))
    assert not is_joint_feedback_frame(
        struct.pack('=IB3x8s', 0x800002A3, 8, bytes(8)))
    assert not is_joint_feedback_frame(frame[:8])


def test_continuous_feedback_has_no_gap():
    tracker = GapTracker(start_monotonic=0.0, gap_threshold_sec=1.0)
    for now in (0.1, 0.4, 0.8, 1.2, 1.5):
        assert tracker.observe(now) is None
        assert not tracker.check(now)
    assert tracker.gap_count == 0


def test_stopped_feedback_is_reported_once_and_recovery_is_measured():
    tracker = GapTracker(start_monotonic=0.0, gap_threshold_sec=1.0)
    tracker.observe(0.2)
    assert not tracker.check(1.1)
    assert tracker.check(1.3)
    assert not tracker.check(1.8)
    assert tracker.observe(2.2) == 2.0
    assert tracker.gap_count == 1
    assert tracker.max_gap_sec == 2.0


def test_no_feedback_at_start_is_a_gap():
    tracker = GapTracker(start_monotonic=5.0, gap_threshold_sec=1.0)
    assert tracker.check(6.0)
    assert tracker.age_sec(6.0) == 1.0


def test_manual_stop_prints_summary_without_traceback(monkeypatch, capsys):
    class InterruptingBus:
        def __enter__(self):
            return self

        def __exit__(self, *_args):
            return False

        def bind(self, _address):
            pass

        def settimeout(self, _seconds):
            pass

        def recv(self, _size):
            raise KeyboardInterrupt

    monkeypatch.setattr(socket, 'socket', lambda *_args: InterruptingBus())
    assert monitor('can0', 10.0, 1.0, 1.0) == 130
    output = capsys.readouterr()
    assert 'INTERRUPTED' in output.out
    assert 'Traceback' not in output.err
