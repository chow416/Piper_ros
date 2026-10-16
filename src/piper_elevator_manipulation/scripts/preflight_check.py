#!/usr/bin/env python3
"""Check that nothing else is already driving the arm before you start a run.

WHY THIS EXISTS
---------------
A stale `ros2_control_node` from an earlier session is invisible until it ruins a
run. Two controller managers both write joint commands to can0 every 5 ms, and
the arm then receives two contradictory setpoints per cycle. The symptom looks
like a control problem -- the arm responds sluggishly, lags tens of degrees
behind, and never reaches the target -- when the real cause is duplicate
processes.

That is not hypothetical: it happened on 2026-09-24. A leftover controller
manager held the home pose while a second one ran the press trajectory, and the
per-joint tracking report showed `joint5 commanded -35.677 deg, measured
+0.000 deg`, with the arm sitting exactly at home the whole time.

The same applies to `move_group`: several instances mean the planner you talk to
is not the one whose planning scene you inspected.

Reads /proc only, so it works without a working ROS graph and cannot itself
disturb the arm.

    python3 scripts/preflight_check.py
    ros2 run piper_elevator_manipulation preflight_check.py
"""

import argparse
import os
import re
import subprocess
import sys
import time
from pathlib import Path

# /proc/<pid>/comm holds the executable name, truncated to 15 characters, so
# these are prefixes rather than full names. Matching on comm instead of the
# full command line avoids the trap where `pkill -f <pattern>` matches the shell
# that is running it.
SINGLETON_PROCESSES = (
    ("ros2_control_no", "ros2_control_node", 1),
    ("move_group", "move_group", 1),
    ("press_button_de", "press_button_demo", 1),
    ("rviz2", "rviz2", 1),
)


def process_state(pid):
    """Single-letter process state from /proc/<pid>/stat, or None.

    The comm field is wrapped in parentheses and may itself contain spaces and
    parentheses, so the reliable split is on the *last* ')'.
    """
    try:
        stat = Path(f"/proc/{pid}/stat").read_text()
        return stat.rsplit(")", 1)[1].split()[0]
    except (OSError, IndexError):
        return None


def scan_processes():
    """Map the executable name to the list of (pid, state, full command line)."""
    found = {}
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit():
            continue
        try:
            comm = (entry / "comm").read_text().strip()
        except (OSError, PermissionError):
            continue
        state = process_state(entry.name)
        if state in ("Z", "X", "x"):
            # Exited but not yet reaped: it owns no CAN socket and answers no
            # service call, so it must not be counted as "in use".
            continue
        for prefix, _label, _limit in SINGLETON_PROCESSES:
            if comm.startswith(prefix):
                try:
                    cmdline = ((entry / "cmdline").read_bytes()
                               .decode(errors="replace").replace("\0", " ").strip())
                except OSError:
                    cmdline = ""
                found.setdefault(prefix, []).append(
                    (entry.name, state, cmdline))
    return found


def count_joint_state_publishers():
    """Ground truth for duplicates: how many nodes publish /joint_states."""
    try:
        completed = subprocess.run(
            ["ros2", "topic", "info", "/joint_states"],
            capture_output=True, text=True, timeout=15)
    except (OSError, subprocess.SubprocessError):
        return None
    if completed.returncode != 0:
        return None
    for line in completed.stdout.splitlines():
        if line.startswith("Publisher count:"):
            try:
                return int(line.split(":", 1)[1].strip())
            except ValueError:
                return None
    return None


# IFF_UP: the administrative "interface enabled" bit. `ip link set can0 up`
# sets it; every replug and reboot clears it.
IFF_UP = 0x1


def can_interface_state(name):
    """Return "up", "down", or None when the interface does not exist.

    `operstate` is deliberately not used. A CAN interface has no carrier to
    detect, so its operstate is not a dependable signal, and the values do not
    line up with what actually matters: docker0 reads `down` while being
    administratively up, and a running CAN interface reports `unknown`.

    The IFF_UP bit is exactly what `ip link set can0 up` toggles, and it is the
    one fact that decides whether the arm can be reached at all.
    """
    try:
        flags = int(Path(f"/sys/class/net/{name}/flags").read_text().strip(), 16)
    except (OSError, ValueError):
        return None
    return "up" if flags & IFF_UP else "down"


def read_can_rx_packets(name):
    """Read the kernel RX packet counter for one CAN interface."""
    try:
        return int(Path(
            f"/sys/class/net/{name}/statistics/rx_packets").read_text().strip())
    except (OSError, ValueError):
        return None


def can_receives_frames(name, sample_seconds=0.25):
    """Return whether the CAN RX counter advances during a short sample."""
    before = read_can_rx_packets(name)
    time.sleep(sample_seconds)
    after = read_can_rx_packets(name)
    live = before is not None and after is not None and after > before
    return live, before, after


ANSI_ESCAPE = re.compile(r"\x1b\[[0-?]*[ -/]*[@-~]")


def parse_hardware_component_state(output, component_name):
    """Extract a ros2_control hardware state from list-components output."""
    clean = ANSI_ESCAPE.sub("", output)
    current_name = None
    for line in clean.splitlines():
        stripped = line.strip()
        if stripped.startswith("name:"):
            current_name = stripped.split(":", 1)[1].strip()
        elif current_name == component_name and stripped.startswith("state:"):
            match = re.search(r"\blabel=([^\s]+)", stripped)
            return match.group(1) if match else None
    return None


def hardware_component_state(component_name):
    """Ask controller_manager for one hardware component's lifecycle state."""
    try:
        completed = subprocess.run(
            ["ros2", "control", "list_hardware_components", "-v"],
            capture_output=True, text=True, timeout=15)
    except (OSError, subprocess.SubprocessError):
        return None
    if completed.returncode != 0:
        return None
    return parse_hardware_component_state(completed.stdout, component_name)


def main():
    parser = argparse.ArgumentParser(
        description="Check for stale or duplicated ROS processes before a run.")
    parser.add_argument("--can", default="can0")
    parser.add_argument("--skip-topic-check", action="store_true",
                        help="skip the /joint_states publisher count")
    parser.add_argument("--skip-can-traffic-check", action="store_true",
                        help="skip the short live CAN RX counter sample")
    parser.add_argument("--skip-hardware-check", action="store_true",
                        help="skip the PiperSystem lifecycle-state query")
    arguments = parser.parse_args()

    found = scan_processes()
    problems = []

    print("=== process check ===")
    for prefix, label, limit in SINGLETON_PROCESSES:
        entries = found.get(prefix, [])
        state = "ok" if len(entries) <= limit else "DUPLICATE"
        print(f"  {label:<22} {len(entries)} (allowed <= {limit})   {state}")
        for pid, pstate, cmdline in entries:
            print(f"      pid {pid} [{pstate}]: {cmdline[:100]}")
        if len(entries) > limit:
            problems.append(
                f"{len(entries)} x {label}; kill the stale ones with:\n"
                f"      kill -9 {' '.join(pid for pid, _, _ in entries)}\n"
                "    then restart the bringup -- the arm loses power when a "
                "controller manager exits, so run go_home afterwards.")

    print()
    print("=== can interface ===")
    state = can_interface_state(arguments.can)
    if state is None:
        print(f"  {arguments.can}: not present")
        problems.append(f"{arguments.can} does not exist; check the USB-CAN adapter")
    elif state == "down":
        print(f"  {arguments.can}: DOWN  <-- the arm cannot be reached")
        problems.append(
            f"{arguments.can} is DOWN. Bring it up first -- a USB-CAN adapter\n"
            "    comes back down after every replug and reboot, and the bitrate\n"
            "    has to be set again:\n"
            f"      sudo ip link set {arguments.can} up type can bitrate 1000000\n"
            "    Without this the bringup blocks waiting for feedback and never\n"
            "    prints `Arm feedback received`.")
    else:
        print(f"  {arguments.can}: up")
        if not arguments.skip_can_traffic_check:
            live, before, after = can_receives_frames(arguments.can)
            delta = None if before is None or after is None else after - before
            if live:
                print(f"  live RX: yes (+{delta} packets / 0.25 s)")
            else:
                print(f"  live RX: NO (counter {before} -> {after})")
                problems.append(
                    f"{arguments.can} is UP but receives no live frames. The "
                    "USB-CAN interface may be stalled; restart can0 (or replug "
                    "the adapter), then verify candump before any motion")

    controller_managers = len(found.get("ros2_control_no", []))
    if not arguments.skip_hardware_check and controller_managers == 1:
        print()
        print("=== ros2_control hardware ===")
        hardware_state = hardware_component_state("PiperSystem")
        if hardware_state == "active":
            print("  PiperSystem: active")
        else:
            print(f"  PiperSystem: {hardware_state or 'unavailable'}  <-- NOT READY")
            problems.append(
                "PiperSystem is not active. Do not publish a trajectory: stop "
                "the stale bringup and start piper_real_bringup again after "
                "live CAN feedback has recovered")

    if not arguments.skip_topic_check:
        print()
        print("=== /joint_states publishers (ground truth) ===")
        publishers = count_joint_state_publishers()
        if publishers is None:
            print("  unavailable (no running graph, or ros2 CLI not on PATH)")
        else:
            print(f"  publisher count: {publishers}")
            if publishers > 1:
                problems.append(
                    f"{publishers} nodes publish /joint_states; more than one "
                    "controller manager is running")

    print()
    if problems:
        print("=== NOT CLEAN ===")
        for index, problem in enumerate(problems, 1):
            print(f"  {index}. {problem}")
        return 1

    print("=== CLEAN ===")
    print("  safe to start a run")
    return 0


if __name__ == "__main__":
    sys.exit(main())
