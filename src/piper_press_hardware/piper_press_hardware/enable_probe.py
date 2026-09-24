"""Guarded, single-motor enable-only test; never sends a position command.

This is a commissioning probe, not a trajectory controller. It deliberately
cannot select motor 6 or the SDK's all-motors selector 7.
"""

import argparse
import math
import time

from .feedback import InvalidFeedback, decode_joint_feedback


MOTORS = (1, 2, 3, 4, 5)
MAX_AGE_SEC = 0.25
MAX_ENABLE_SHIFT_RAD = 0.02
GRAVITY_JOINT_MARGIN_RAD = 0.05


class ProbeRejected(ValueError):
    """A commissioning precondition or feedback check failed."""


class TransientFeedback(ProbeRejected):
    """Feedback may not have populated yet immediately after connection."""


def _fresh_timestamp(value, now_sec, label):
    timestamp = float(value)
    age = now_sec - timestamp
    if not math.isfinite(age) or age < -0.05 or age > MAX_AGE_SEC:
        raise TransientFeedback(f'{label} is missing or stale (age={age:.3f}s)')
    return timestamp


def _snapshot(sdk):
    """Return fresh joint positions and all six motor-enable flags."""
    if not sdk.isOk():
        raise TransientFeedback('SDK CAN receive thread is not healthy')
    now = time.time()
    try:
        joints = decode_joint_feedback(sdk.GetArmJointMsgs(), now, MAX_AGE_SEC, 3.5)
        status = sdk.GetArmStatus()
        _fresh_timestamp(status.time_stamp, now, 'arm status')
        if int(status.arm_status.err_code) != 0:
            raise ProbeRejected(f'arm error code={status.arm_status.err_code}')
        if int(status.arm_status.arm_status) != 0:
            raise ProbeRejected(f'arm is not in NORMAL state: '
                                f'{status.arm_status.arm_status}')
        if int(status.arm_status.ctrl_mode) not in (0, 1):
            raise ProbeRejected(f'unexpected arm control mode: '
                                f'{status.arm_status.ctrl_mode}')
        if int(status.arm_status.teach_status) != 0:
            raise ProbeRejected('teaching is active')
        low_speed = sdk.GetArmLowSpdInfoMsgs()
        low_stamp = _fresh_timestamp(low_speed.time_stamp, now, 'motor status')
        flags = []
        for index in range(1, 7):
            motor = getattr(low_speed, f'motor_{index}')
            if int(motor.can_id) != 0x260 + index:
                raise TransientFeedback(f'J{index} has no valid motor-status frame')
            flags.append(bool(motor.foc_status.driver_enable_status))
        return joints.positions_rad, flags, low_stamp
    except ProbeRejected:
        raise
    except InvalidFeedback as error:
        raise TransientFeedback(str(error)) from error
    except (AttributeError, TypeError, ValueError) as error:
        raise ProbeRejected(str(error)) from error


def _wait_for_flag(sdk, motor, expected, previous_stamp, timeout_sec):
    deadline = time.monotonic() + timeout_sec
    last_reason = 'no new motor status'
    while time.monotonic() < deadline:
        try:
            positions, flags, stamp = _snapshot(sdk)
            if flags[5]:
                raise ProbeRejected('J6 became enabled; use independent stop')
            if any(flags[index - 1] for index in MOTORS if index != motor):
                raise ProbeRejected('another motor became enabled; use independent stop')
            if stamp > previous_stamp and flags[motor - 1] == expected:
                return positions, stamp
            last_reason = f'J{motor} enable feedback={flags[motor - 1]}'
        except TransientFeedback as error:
            # Missing CAN feedback may recover within the short timeout.
            last_reason = str(error)
        time.sleep(0.05)
    raise ProbeRejected(f'timed out waiting for J{motor}={expected}: {last_reason}')


def _check_gravity_joint_start(positions_rad):
    """Keep J2/J3 enable tests away from uncertain zero/limit boundaries."""
    from .hardware_preflight import _load_press_limits

    try:
        limits = _load_press_limits()
    except (OSError, ValueError) as error:
        raise ProbeRejected(f'cannot load press-tool model: {error}') from error
    for motor in (2, 3):
        position = positions_rad[motor - 1]
        bound = limits[motor - 1]
        lower = bound.lower_rad + GRAVITY_JOINT_MARGIN_RAD
        upper = bound.upper_rad - GRAVITY_JOINT_MARGIN_RAD
        if not lower <= position <= upper:
            raise ProbeRejected(
                f'J{motor}={position:+.5f} rad is not at least '
                f'{GRAVITY_JOINT_MARGIN_RAD:.2f} rad inside the press URDF '
                f'limits [{bound.lower_rad:+.5f}, {bound.upper_rad:+.5f}]; '
                'no enable command sent')


def run(sdk, motor, execute=False, hold_sec=1.0, feedback_timeout_sec=2.0):
    """Inspect all motors, then optionally pulse one J1–J5 enable.

    The caller owns the CAN connection. On an executed test, a best-effort
    DisableArm is sent only for the selected motor in ``finally``. This does
    not substitute for an independent physical stop or support against falls.
    """
    if motor not in MOTORS:
        raise ValueError('motor must be one of 1, 2, 3, 4, 5')
    if not 0.1 <= hold_sec <= 2.0:
        raise ValueError('hold_sec must be between 0.1 and 2.0')
    if not 0.2 <= feedback_timeout_sec <= 5.0:
        raise ValueError('feedback_timeout_sec must be between 0.2 and 5.0')

    initial_positions, flags, stamp = _snapshot(sdk)
    print(f'PRECHECK motor_flags_J1_to_J6={flags}', flush=True)
    if any(flags):
        raise ProbeRejected('one or more motors are already enabled; no command sent')
    if motor in (2, 3):
        _check_gravity_joint_start(initial_positions)
    if not execute:
        print(f'DRY_RUN PASS: J{motor} selected; no enable/disable/motion command sent',
              flush=True)
        return

    print(f'ENABLE_TEST J{motor}: one enable, no mode change or JointCtrl', flush=True)
    enable_attempted = False
    try:
        # Treat even a raised SDK send as uncertain delivery: still attempt
        # the matching disable when the CAN session is healthy enough.
        enable_attempted = True
        sdk.EnableArm(motor_num=motor)
        enabled_positions, stamp = _wait_for_flag(
            sdk, motor, True, stamp, feedback_timeout_sec)
        shift = max(abs(after - before) for before, after in
                    zip(initial_positions, enabled_positions))
        if shift > MAX_ENABLE_SHIFT_RAD:
            raise ProbeRejected(f'arm shifted {shift:.4f} rad during enable')
        print(f'J{motor} enable feedback=True; max joint shift={shift:.4f} rad',
              flush=True)
        hold_deadline = time.monotonic() + hold_sec
        while time.monotonic() < hold_deadline:
            time.sleep(max(0.0, min(0.05, hold_deadline - time.monotonic())))
            held_positions, held_flags, _ = _snapshot(sdk)
            if not held_flags[motor - 1]:
                raise ProbeRejected(f'J{motor} unexpectedly disabled during hold')
            if held_flags[5] or any(held_flags[index - 1] for index in MOTORS
                                    if index != motor):
                raise ProbeRejected('another motor became enabled; use independent stop')
            held_shift = max(abs(after - before) for before, after in
                             zip(initial_positions, held_positions))
            if held_shift > MAX_ENABLE_SHIFT_RAD:
                raise ProbeRejected(f'arm shifted {held_shift:.4f} rad while enabled')
    finally:
        if enable_attempted:
            print(f'DISABLE_TEST J{motor}: best-effort CAN command; keep physical support',
                  flush=True)
            sdk.DisableArm(motor_num=motor)
    _, _ = _wait_for_flag(sdk, motor, False, stamp, feedback_timeout_sec)
    _, final_flags, _ = _snapshot(sdk)
    if any(final_flags):
        raise ProbeRejected(f'one or more motors remain enabled: {final_flags}')
    print(f'PASS J{motor} enable/disable feedback; no trajectory sent', flush=True)


def main():
    parser = argparse.ArgumentParser(description='Guarded single-J1–J5 enable test')
    parser.add_argument('--can-port', default='can0')
    parser.add_argument('--motor', type=int, choices=MOTORS, required=True)
    parser.add_argument('--execute', action='store_true',
                        help='actually send one enable and one disable command')
    args = parser.parse_args()
    try:
        from piper_sdk import C_PiperInterface
        sdk = C_PiperInterface(can_name=args.can_port)
        connected = False
        try:
            sdk.ConnectPort(piper_init=False)
            connected = True
            deadline = time.monotonic() + 3.0
            while True:
                try:
                    run(sdk, args.motor, execute=False)
                    break
                except TransientFeedback:
                    if time.monotonic() >= deadline:
                        raise
                    time.sleep(0.05)
            if args.execute:
                run(sdk, args.motor, execute=True)
            return 0
        finally:
            if connected:
                sdk.DisconnectPort()
    except (ImportError, OSError, RuntimeError, ProbeRejected, KeyboardInterrupt) as error:
        print(f'FAIL enable probe: {error}', flush=True)
        if args.execute:
            print('MOTOR STATE UNKNOWN after failure/interruption: verify physical '
                  'state and use the independent stop if needed', flush=True)
        return 1
