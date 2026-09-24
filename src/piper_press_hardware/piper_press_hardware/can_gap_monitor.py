"""Receive-only CAN feedback soak test for intermittent USB-CAN failures."""

import argparse
from dataclasses import dataclass
from datetime import datetime
import socket
import struct
import time
from typing import Optional


# Three joint-state CAN frames are sufficient to establish live arm feedback.
# A different device on the bus cannot mask their loss by sending other IDs.
JOINT_FEEDBACK_IDS = frozenset((0x2A3, 0x2A4, 0x2A5))
CAN_STANDARD_ID_MASK = 0x7FF
CAN_FRAME_FLAGS = 0xE0000000  # Extended, remote-request, or error frame.


def is_joint_feedback_frame(frame: bytes) -> bool:
    """Accept only standard, data-bearing Piper joint feedback frames."""
    if len(frame) < 16:
        return False
    raw_can_id = struct.unpack_from('=I', frame)[0]
    return ((raw_can_id & CAN_FRAME_FLAGS) == 0 and
            (raw_can_id & CAN_STANDARD_ID_MASK) in JOINT_FEEDBACK_IDS)


@dataclass
class GapTracker:
    """Detect a loss and later recovery of selected feedback frames."""

    start_monotonic: float
    gap_threshold_sec: float
    last_frame_monotonic: Optional[float] = None
    gap_start_monotonic: Optional[float] = None
    gap_count: int = 0
    max_gap_sec: float = 0.0

    def check(self, now_monotonic: float) -> bool:
        """Return True only when a new gap exceeds the threshold."""
        last = self.last_frame_monotonic
        if last is None:
            last = self.start_monotonic
        if (self.gap_start_monotonic is None and
                now_monotonic - last >= self.gap_threshold_sec):
            self.gap_start_monotonic = last
            self.gap_count += 1
            return True
        if self.gap_start_monotonic is not None:
            self.max_gap_sec = max(
                self.max_gap_sec, now_monotonic - self.gap_start_monotonic)
        return False

    def observe(self, now_monotonic: float):
        """Record one selected frame; return the finished gap duration, if any."""
        gap_duration = None
        if self.gap_start_monotonic is not None:
            gap_duration = now_monotonic - self.gap_start_monotonic
            self.max_gap_sec = max(self.max_gap_sec, gap_duration)
            self.gap_start_monotonic = None
        self.last_frame_monotonic = now_monotonic
        return gap_duration

    def age_sec(self, now_monotonic: float) -> float:
        last = self.last_frame_monotonic
        return now_monotonic - (last if last is not None else self.start_monotonic)


def _log(message: str):
    print(f'{datetime.now().astimezone().isoformat(timespec="seconds")} '
          f'{message}', flush=True)


def monitor(can_port: str, duration_sec: float, gap_threshold_sec: float,
            report_interval_sec: float) -> int:
    """Read Linux SocketCAN frames without transmitting any CAN command."""
    start = time.monotonic()
    tracker = GapTracker(start, gap_threshold_sec)
    deadline = start + duration_sec
    last_report = start
    report_frames = 0
    total_frames = 0
    interrupted = False

    try:
        with socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW) as bus:
            bus.bind((can_port,))
            bus.settimeout(0.1)
            _log(f'START can={can_port} duration={duration_sec:.0f}s '
                 f'gap_limit={gap_threshold_sec:.2f}s '
                 'receive_only=true ids=2A3,2A4,2A5')

            while time.monotonic() < deadline:
                try:
                    frame = bus.recv(16)
                except socket.timeout:
                    frame = b''
                now = time.monotonic()

                # Check before recording a newly arrived frame so a long
                # pause remains visible even if the first read is recovery.
                if tracker.check(now):
                    _log(f'GAP_START last_frame_age={tracker.age_sec(now):.2f}s')

                if is_joint_feedback_frame(frame):
                    recovered_gap = tracker.observe(now)
                    total_frames += 1
                    report_frames += 1
                    if recovered_gap is not None:
                        _log(f'RECOVERED gap={recovered_gap:.2f}s')

                if now - last_report >= report_interval_sec:
                    _log(f'HEARTBEAT elapsed={now - start:.0f}s '
                         f'frames={report_frames} '
                         f'last_frame_age={tracker.age_sec(now):.2f}s '
                         f'gaps={tracker.gap_count}')
                    last_report = now
                    report_frames = 0
    except KeyboardInterrupt:
        interrupted = True
    except (OSError, ValueError) as error:
        _log(f'ERROR socketcan={error}')
        return 2

    end = time.monotonic()
    tracker.check(end)
    label = 'INTERRUPTED' if interrupted else 'END'
    _log(f'{label} duration={end - start:.0f}s total_joint_frames={total_frames} '
         f'gaps={tracker.gap_count} max_gap={tracker.max_gap_sec:.2f}s '
         f'last_frame_age={tracker.age_sec(end):.2f}s')
    if interrupted:
        return 130
    return 0 if tracker.gap_count == 0 else 1


def main():
    parser = argparse.ArgumentParser(
        description='Receive-only Piper joint feedback continuity check')
    parser.add_argument('--can-port', default='can0')
    parser.add_argument('--duration-sec', type=float, default=1800.0)
    parser.add_argument('--gap-threshold-sec', type=float, default=1.0)
    parser.add_argument('--report-interval-sec', type=float, default=10.0)
    args = parser.parse_args()
    if not 1.0 <= args.duration_sec <= 86_400.0:
        parser.error('--duration-sec must be between 1 and 86400')
    if not 0.1 <= args.gap_threshold_sec <= args.duration_sec:
        parser.error('--gap-threshold-sec must be between 0.1 and duration')
    if not 1.0 <= args.report_interval_sec <= args.duration_sec:
        parser.error('--report-interval-sec must be between 1 and duration')
    return monitor(args.can_port, args.duration_sec,
                   args.gap_threshold_sec, args.report_interval_sec)
