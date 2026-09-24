"""Supervised, low-speed J1-J5 motion toward joint zero.

This is a one-shot commissioning tool, not the final MoveIt controller. The
Piper SDK's JointCtrl always contains six values, so the sixth value is held
at the observed J6 angle while its motor remains disabled.
"""

import argparse
import math
import time

from .enable_probe import ProbeRejected, TransientFeedback, _snapshot
from .feedback import MILLIDEGREES_TO_RADIANS
from .hardware_preflight import _load_press_limits


MAX_START_OUTSIDE_RAD = 0.05
MAX_TOTAL_TRAVEL_RAD = 0.40
MAX_STEP_RAD = math.radians(0.5)
FIRST_TEST_TRAVEL_RAD = math.radians(1.0)
MAX_ENABLE_SHIFT_RAD = 0.02
MAX_WAYPOINT_ERROR_RAD = 0.035
TARGET_TOLERANCE_RAD = 0.003
SPEED_PERCENT = 5
STREAM_PERIOD_SEC = 0.02
WAYPOINT_TIMEOUT_SEC = 3.0
STABLE_FEEDBACK_SAMPLES = 3


def _j6_angle_mdeg(sdk):
    message = sdk.GetArmJointMsgs()
    age = time.time() - float(message.time_stamp)
    if not math.isfinite(age) or age < -0.05 or age > 0.25:
        raise ProbeRejected('J6 joint feedback is missing or stale')
    raw = int(message.joint_state.joint_6)
    if abs(raw * MILLIDEGREES_TO_RADIANS) > 2.0944:
        raise ProbeRejected('J6 joint angle is implausible')
    return raw


def make_zero_plan(current_rad, limits, full_home=False):
    """Return bounded J1-J5 setpoints from the current pose to zero.

    Only a small J2/J3 zero-boundary overrun is allowed at the initial sample.
    Every transmitted setpoint is inside the press-tool URDF limits.
    """
    if len(current_rad) != 5 or len(limits) != 5:
        raise ProbeRejected('expected five current joints and five limits')
    start = []
    for index, (position, bound) in enumerate(zip(current_rad, limits), 1):
        if not math.isfinite(position):
            raise ProbeRejected(f'J{index} feedback is non-finite')
        outside = max(bound.lower_rad - position, position - bound.upper_rad, 0.0)
        allowance = MAX_START_OUTSIDE_RAD if index in (2, 3) else 0.0
        if outside > allowance:
            raise ProbeRejected(f'J{index} exceeds URDF limit by {outside:.4f} rad')
        if abs(position) > MAX_TOTAL_TRAVEL_RAD:
            raise ProbeRejected(f'J{index} is too far from zero for this probe')
        if not bound.lower_rad <= 0.0 <= bound.upper_rad:
            raise ProbeRejected(f'J{index} zero is outside URDF limits')
        start.append(min(max(position, bound.lower_rad), bound.upper_rad))
    if full_home:
        goal = (0.0,) * 5
    else:
        goal = list(start)
        goal[0] -= math.copysign(min(abs(start[0]), FIRST_TEST_TRAVEL_RAD),
                                 start[0])
        goal = tuple(goal)
    count = max(1, math.ceil(max(abs(a - b) for a, b in zip(start, goal)) /
                             MAX_STEP_RAD))
    waypoints = [tuple(a + (b - a) * step / count
                       for a, b in zip(start, goal))
                 for step in range(1, count + 1)]
    return tuple(start), tuple(waypoints)


def _joint_ctrl_values(positions_rad, j6_mdeg):
    values = [int(round(q / MILLIDEGREES_TO_RADIANS)) for q in positions_rad]
    return (*values, j6_mdeg)


def _wait_for_enabled_set(sdk, expected, initial_rad, timeout_sec=2.0):
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        positions, flags, _ = _snapshot(sdk)
        if flags[5]:
            raise ProbeRejected('J6 unexpectedly enabled')
        if tuple(flags[:5]) == expected:
            shift = max(abs(a - b) for a, b in zip(positions, initial_rad))
            if shift > MAX_ENABLE_SHIFT_RAD:
                raise ProbeRejected(f'arm shifted {shift:.4f} rad during enable')
            return
        time.sleep(0.05)
    raise ProbeRejected(f'waiting for motor flags timed out: expected {expected}')


def _wait_for_mode(sdk, control_mode, timeout_sec=2.0):
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        _snapshot(sdk)
        status = sdk.GetArmStatus()
        age = time.time() - float(status.time_stamp)
        if not math.isfinite(age) or age < -0.05 or age > 0.25:
            raise ProbeRejected('arm mode feedback is stale')
        if (int(status.arm_status.ctrl_mode) == control_mode and
                (control_mode == 0 or int(status.arm_status.mode_feed) == 1)):
            return
        time.sleep(0.05)
    raise ProbeRejected(f'control mode {control_mode} did not appear in feedback')


def _stream_target(sdk, target_rad, j6_mdeg, active_motors,
                   timeout_sec=WAYPOINT_TIMEOUT_SEC):
    """Refresh a bounded MOVE_J target until fresh feedback confirms arrival."""
    deadline = time.monotonic() + timeout_sec
    error = math.inf
    best_error = math.inf
    samples_at_target = 0
    commands_sent = 0
    while time.monotonic() < deadline:
        sdk.JointCtrl(*_joint_ctrl_values(target_rad, j6_mdeg))
        commands_sent += 1
        positions, flags, _ = _snapshot(sdk)
        expected_flags = tuple(index in active_motors for index in range(1, 7))
        if tuple(flags) != expected_flags:
            raise ProbeRejected(f'unexpected motor-enable state {flags}')
        error = max(abs(positions[index - 1] - target_rad[index - 1])
                    for index in active_motors)
        best_error = min(best_error, error)
        if error <= TARGET_TOLERANCE_RAD:
            samples_at_target += 1
            if samples_at_target >= STABLE_FEEDBACK_SAMPLES:
                return commands_sent
        else:
            samples_at_target = 0
        if error > MAX_WAYPOINT_ERROR_RAD:
            raise ProbeRejected(f'waypoint tracking error {error:.4f} rad')
        time.sleep(STREAM_PERIOD_SEC)
    raise ProbeRejected(f'waypoint tracking timed out (final_error={error:.4f} rad, '
                        f'best_error={best_error:.4f} rad, '
                        f'commands_sent={commands_sent})')


def _wait_for_disabled(sdk, timeout_sec=2.0):
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        _, flags, _ = _snapshot(sdk)
        if not any(flags):
            return
        time.sleep(0.05)
    raise ProbeRejected(f'motor(s) remain enabled: {flags}')


def _cleanup_after_motion(sdk, attempted_motors, mode_attempted):
    """Best-effort cleanup that never hides the original motion failure."""
    problems = []
    for motor in reversed(attempted_motors):
        try:
            sdk.DisableArm(motor_num=motor)
        except Exception as error:
            problems.append(f'J{motor} disable send failed: {error}')
    disabled_confirmed = False
    try:
        _wait_for_disabled(sdk)
        disabled_confirmed = True
        print('CLEANUP motor_flags_J1_to_J6=all_disabled', flush=True)
    except Exception as error:
        problems.append(f'disabled feedback not confirmed: {error}')
    if mode_attempted and disabled_confirmed:
        try:
            sdk.MotionCtrl_2(0x00, 0x01, SPEED_PERCENT, 0x00)
            _wait_for_mode(sdk, 0)
            print('CLEANUP control_mode=STANDBY', flush=True)
        except Exception as error:
            problems.append(f'STANDBY feedback not confirmed: {error}')
    elif mode_attempted:
        problems.append('STANDBY command withheld until all motors are confirmed disabled')
    for problem in problems:
        print(f'CLEANUP WARNING: {problem}', flush=True)
    return problems


def recover_standby(sdk):
    """Restore STANDBY only after confirming every motor is disabled."""
    _, flags, _ = _snapshot(sdk)
    if any(flags):
        raise ProbeRejected(f'recovery refused: motor(s) enabled: {flags}')
    status = sdk.GetArmStatus()
    if int(status.arm_status.ctrl_mode) == 0:
        print('RECOVERY PASS: already STANDBY; all motors disabled', flush=True)
        return
    if (int(status.arm_status.ctrl_mode) != 1 or
            int(status.arm_status.mode_feed) != 1):
        raise ProbeRejected('recovery refused: expected CAN_CTRL/MOVE_J')
    sdk.MotionCtrl_2(0x00, 0x01, SPEED_PERCENT, 0x00)
    _wait_for_mode(sdk, 0)
    _wait_for_disabled(sdk)
    print('RECOVERY PASS: STANDBY; all motors disabled', flush=True)


def run(sdk, execute=False, full_home=False):
    """Preview or execute a single supervised zero-directed motion."""
    current_rad, flags, _ = _snapshot(sdk)
    if any(flags):
        raise ProbeRejected('all six motors must initially be disabled')
    status = sdk.GetArmStatus()
    if int(status.arm_status.ctrl_mode) != 0:
        raise ProbeRejected('expected STANDBY before the first motion')
    if int(status.arm_status.mode_feed) not in (0, 1):
        raise ProbeRejected('unexpected prior motion mode; do not switch from MIT/teach')
    limits = _load_press_limits()
    start, waypoints = make_zero_plan(current_rad, limits, full_home=full_home)
    if execute and full_home and any(abs(a - b) > TARGET_TOLERANCE_RAD
                                     for a, b in zip(current_rad, start)):
        raise ProbeRejected('full-home execution requires every starting joint '
                            'inside its URDF limits; no command sent')
    j6_mdeg = _j6_angle_mdeg(sdk)
    active_motors = tuple(range(1, 6)) if full_home else (1,)
    clamp_deg = tuple(round(abs(math.degrees(a - b)), 2)
                      for a, b in zip(current_rad, start))
    command_delta_deg = tuple(round(abs(math.degrees(a - b)), 2)
                              for a, b in zip(start, waypoints[-1]))
    print(f'ZERO_PREVIEW current_J1_to_J5_rad={tuple(round(q, 5) for q in current_rad)}',
          flush=True)
    print(f'ZERO_PREVIEW full_zero_distance_deg='
          f'{tuple(round(abs(math.degrees(q)), 2) for q in current_rad)} '
          f'sdk_setpoint_clamp_deg={clamp_deg}', flush=True)
    print(f'ZERO_PREVIEW mode={"FULL_HOME" if full_home else "FIRST_1_DEG_TEST"} '
          f'active_motors={active_motors} '
          f'initial_in_limits_rad={tuple(round(q, 5) for q in start)} '
          f'goal_rad={tuple(round(q, 5) for q in waypoints[-1])} '
          f'planned_command_delta_deg={command_delta_deg} '
          f'waypoints={len(waypoints)} speed_percent={SPEED_PERCENT} '
          f'J6_disabled_hold_mdeg={j6_mdeg}', flush=True)
    if not execute:
        print('DRY_RUN PASS: no CAN command sent', flush=True)
        return

    enabled = []
    mode_attempted = False
    cleanup_problems = []
    try:
        # A valid hold setpoint is staged before enabling a motor. The small
        # J2/J3 boundary correction is bounded by MAX_START_OUTSIDE_RAD.
        mode_attempted = True
        sdk.MotionCtrl_2(0x01, 0x01, SPEED_PERCENT, 0x00)
        _wait_for_mode(sdk, 1)
        sdk.JointCtrl(*_joint_ctrl_values(start, j6_mdeg))
        for motor in active_motors:
            enabled.append(motor)  # uncertain send still requires disable attempt
            sdk.EnableArm(motor_num=motor)
            expected = tuple(index in enabled for index in range(1, 6))
            _wait_for_enabled_set(sdk, expected, current_rad)
            print(f'J{motor} enable feedback=True', flush=True)
        for index, waypoint in enumerate(waypoints, 1):
            commands_sent = _stream_target(sdk, waypoint, j6_mdeg, active_motors)
            print(f'WAYPOINT {index}/{len(waypoints)} reached '
                  f'(commands_sent={commands_sent})', flush=True)
        print('ZERO_MOTION_REACHED: commanded goal reached; disabling with support',
              flush=True)
    finally:
        cleanup_problems = _cleanup_after_motion(sdk, enabled, mode_attempted)
        if enabled:
            print('Motors may drop as they disable; maintain physical support', flush=True)
    if cleanup_problems:
        raise ProbeRejected('motion reached but cleanup was not fully confirmed')
    print('PASS zero-motion test; all six motors report disabled', flush=True)


def main():
    parser = argparse.ArgumentParser(description='Supervised zero-directed motion test')
    parser.add_argument('--can-port', default='can0')
    parser.add_argument('--execute', action='store_true',
                        help='send low-speed, monitored CAN motion commands')
    parser.add_argument('--full-home', action='store_true',
                        help='plan/execute full zero instead of the first 1-degree test')
    parser.add_argument('--recover-standby', action='store_true',
                        help='with all motors disabled, restore STANDBY after a failed run')
    parser.add_argument('--confirm-clear-and-supported', action='store_true',
                        help='confirm workspace is clear and arm is supported')
    args = parser.parse_args()
    if args.recover_standby and (args.execute or args.full_home):
        parser.error('--recover-standby cannot be combined with motion options')
    if args.execute and not args.confirm_clear_and_supported:
        parser.error('--execute requires --confirm-clear-and-supported')

    connected = False
    try:
        from piper_sdk import C_PiperInterface
        sdk = C_PiperInterface(can_name=args.can_port)
        sdk.ConnectPort(piper_init=False)
        connected = True
        deadline = time.monotonic() + 3.0
        while True:
            try:
                if args.recover_standby:
                    recover_standby(sdk)
                else:
                    run(sdk, execute=False, full_home=args.full_home)
                break
            except TransientFeedback:
                if time.monotonic() >= deadline:
                    raise
                time.sleep(0.05)
        if args.execute:
            run(sdk, execute=True, full_home=args.full_home)
        return 0
    except (ImportError, OSError, RuntimeError, ValueError, KeyboardInterrupt) as error:
        print(f'FAIL zero-home probe: {error}', flush=True)
        if args.execute:
            print('AFTER FAILURE: require CLEANUP all_disabled and STANDBY '
                  'confirmation plus a fresh read-only check before further '
                  'commands; otherwise treat motor state as unknown', flush=True)
        return 1
    finally:
        if connected:
            sdk.DisconnectPort()
