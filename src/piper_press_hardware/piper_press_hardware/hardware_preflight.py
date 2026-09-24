"""One-shot, receive-only check of the real arm against the press-tool URDF."""

import argparse
import math
import time

from .feedback import JOINT_NAMES, InvalidFeedback, decode_joint_feedback
from .model_limits import joint_limit_violations, parse_press_joint_limits


def _load_press_limits():
    # Delay ROS imports so the model/trajectory unit tests stay pure Python.
    from ament_index_python.packages import get_package_share_path
    import xacro

    try:
        path = get_package_share_path('piper_description') / 'urdf' / 'piper_description.xacro'
        model = xacro.process_file(str(path), mappings={'use_press_tool': 'true'})
        return parse_press_joint_limits(model.toxml())
    except Exception as error:
        raise ValueError(f'cannot load press-tool URDF: {error}') from error


def run(can_port: str, wait_sec: float, max_feedback_age_sec: float) -> int:
    """Read feedback only; never call an SDK enable, stop, or motion method."""
    try:
        limits = _load_press_limits()
    except (OSError, ValueError) as error:
        print(f'ERROR model: {error}', flush=True)
        return 2

    try:
        from piper_sdk import C_PiperInterface
    except ImportError as error:
        print(f'ERROR SDK import: {error}', flush=True)
        return 2

    sdk = C_PiperInterface(can_name=can_port)
    connected = False
    print(f'READ_ONLY_PREFLIGHT can={can_port} joints=J1-J5; no CAN commands', flush=True)
    try:
        sdk.ConnectPort(piper_init=False)
        connected = True
        deadline = time.monotonic() + wait_sec
        last_error = 'no fresh SDK feedback'
        while time.monotonic() < deadline:
            if sdk.isOk():
                try:
                    now = time.time()
                    sample = decode_joint_feedback(
                        sdk.GetArmJointMsgs(), now, max_feedback_age_sec, 3.5)
                    status = sdk.GetArmStatus()
                    status_age = now - float(status.time_stamp)
                    error_code = int(status.arm_status.err_code)
                    if (not math.isfinite(status_age) or status_age < -0.05 or
                            status_age > max_feedback_age_sec):
                        last_error = f'arm status age is {status_age:.3f}s'
                    else:
                        break
                except (InvalidFeedback, AttributeError, TypeError, ValueError) as error:
                    last_error = str(error)
            else:
                last_error = 'SDK CAN receive thread is not healthy'
            time.sleep(0.05)
        else:
            print(f'FAIL feedback: {last_error}', flush=True)
            return 1

        for name, position, bound in zip(JOINT_NAMES, sample.positions_rad, limits):
            print(f'{name}: feedback={position:+.5f} rad '
                  f'URDF=[{bound.lower_rad:+.5f}, {bound.upper_rad:+.5f}] rad',
                  flush=True)
        violations = joint_limit_violations(sample.positions_rad, limits)
        if error_code != 0:
            print(f'FAIL arm status: error_code={error_code}', flush=True)
        for violation in violations:
            print(f'FAIL joint limit: {violation}', flush=True)
        if error_code or violations:
            print('No motion permitted; resolve model/feedback discrepancy first.', flush=True)
            return 1
        print('PASS read-only preflight; this does NOT authorize motor motion.', flush=True)
        return 0
    except (OSError, RuntimeError) as error:
        print(f'ERROR CAN/SDK: {error}', flush=True)
        return 2
    finally:
        if connected:
            sdk.DisconnectPort()


def main():
    parser = argparse.ArgumentParser(description='Receive-only J1-J5 model preflight')
    parser.add_argument('--can-port', default='can0')
    parser.add_argument('--wait-sec', type=float, default=3.0)
    parser.add_argument('--max-feedback-age-sec', type=float, default=0.25)
    args = parser.parse_args()
    if not args.can_port:
        parser.error('--can-port must not be empty')
    if not 0.1 <= args.wait_sec <= 30.0:
        parser.error('--wait-sec must be between 0.1 and 30')
    if not 0.01 <= args.max_feedback_age_sec <= 2.0:
        parser.error('--max-feedback-age-sec must be between 0.01 and 2')
    return run(args.can_port, args.wait_sec, args.max_feedback_age_sec)
