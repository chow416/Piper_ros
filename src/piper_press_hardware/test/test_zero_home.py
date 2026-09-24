"""Offline tests; FakeSDK has no CAN connection or physical motors."""

from types import SimpleNamespace
import sys
import time

import pytest

from piper_press_hardware import zero_home
from piper_press_hardware.model_limits import JointLimit


LIMITS = (
    JointLimit(-2.618, 2.618),
    JointLimit(0.0, 3.14),
    JointLimit(-2.967, 0.0),
    JointLimit(-1.745, 1.745),
    JointLimit(-1.22, 1.22),
)


class FakeSDK:
    def __init__(self):
        self.raw = [1719, -2187, 1618, 1800, 20455, 0]
        self.flags = [False] * 6
        self.calls = []
        self.fail_move = False
        self.ctrl_mode = 0
        self.mode_feed = 0

    def isOk(self):
        return True

    def GetArmJointMsgs(self):
        return SimpleNamespace(
            time_stamp=time.time(),
            joint_state=SimpleNamespace(**{
                f'joint_{index}': value
                for index, value in enumerate(self.raw, 1)
            }),
        )

    def GetArmStatus(self):
        return SimpleNamespace(
            time_stamp=time.time(),
            arm_status=SimpleNamespace(
                err_code=0, arm_status=0, ctrl_mode=self.ctrl_mode,
                mode_feed=self.mode_feed, teach_status=0,
            ),
        )

    def GetArmLowSpdInfoMsgs(self):
        return SimpleNamespace(
            time_stamp=time.time(),
            **{
                f'motor_{index}': SimpleNamespace(
                    can_id=0x260 + index,
                    foc_status=SimpleNamespace(driver_enable_status=flag),
                )
                for index, flag in enumerate(self.flags, 1)
            },
        )

    def MotionCtrl_2(self, *args):
        self.calls.append(('mode', args))
        self.ctrl_mode = args[0]
        self.mode_feed = args[1]

    def JointCtrl(self, *values):
        self.calls.append(('joint', values))
        if not self.fail_move:
            for index, enabled in enumerate(self.flags):
                if enabled:
                    self.raw[index] = values[index]

    def EnableArm(self, *, motor_num):
        self.calls.append(('enable', motor_num))
        self.flags[motor_num - 1] = True

    def DisableArm(self, *, motor_num):
        self.calls.append(('disable', motor_num))
        self.flags[motor_num - 1] = False

    def ConnectPort(self, *, piper_init):
        self.calls.append(('connect', piper_init))

    def DisconnectPort(self):
        self.calls.append(('disconnect',))


def test_plan_clamps_only_small_zero_boundary_overrun():
    current = (0.03, -0.038, 0.034, 0.03, 0.34)
    start, points = zero_home.make_zero_plan(current, LIMITS, full_home=True)
    assert start[1:3] == (0.0, 0.0)
    assert points[-1] == (0.0,) * 5
    assert all(max(abs(a - b) for a, b in zip(one, two)) <=
               zero_home.MAX_STEP_RAD + 1e-12
               for one, two in zip((start,) + points[:-1], points))


def test_plan_rejects_large_out_of_limit_and_large_sweep():
    with pytest.raises(zero_home.ProbeRejected, match='J2 exceeds'):
        zero_home.make_zero_plan((0.0, -0.10, 0.0, 0.0, 0.0), LIMITS)
    with pytest.raises(zero_home.ProbeRejected, match='too far'):
        zero_home.make_zero_plan((0.0, 0.0, 0.0, 0.0, 0.50), LIMITS)


def test_default_plan_moves_at_most_one_degree():
    current = (0.03, -0.038, 0.034, 0.03, 0.34)
    start, points = zero_home.make_zero_plan(current, LIMITS)
    assert max(abs(a - b) for a, b in zip(start, points[-1])) <= (
        zero_home.FIRST_TEST_TRAVEL_RAD + 1e-12)
    assert points[-1][0] < start[0]
    assert points[-1][1:] == start[1:]


def test_dry_run_sends_no_command(monkeypatch):
    sdk = FakeSDK()
    monkeypatch.setattr(zero_home, '_load_press_limits', lambda: LIMITS)
    zero_home.run(sdk, execute=False)
    assert sdk.calls == []


def test_execution_uses_five_enables_and_disables_j6_stays_off(monkeypatch):
    sdk = FakeSDK()
    sdk.raw[1:3] = [2187, -1618]
    monkeypatch.setattr(zero_home, '_load_press_limits', lambda: LIMITS)
    zero_home.run(sdk, execute=True, full_home=True)
    assert [call for call in sdk.calls if call[0] == 'enable'] == [
        ('enable', index) for index in range(1, 6)]
    assert [call for call in sdk.calls if call[0] == 'disable'] == [
        ('disable', index) for index in range(5, 0, -1)]
    assert all(call[1][5] == 0 for call in sdk.calls if call[0] == 'joint')
    assert sdk.raw[:5] == [0] * 5
    assert sdk.flags == [False] * 6


def test_full_home_rejects_out_of_limit_start_before_can_command(monkeypatch):
    sdk = FakeSDK()
    monkeypatch.setattr(zero_home, '_load_press_limits', lambda: LIMITS)
    with pytest.raises(zero_home.ProbeRejected, match='inside its URDF limits'):
        zero_home.run(sdk, execute=True, full_home=True)
    assert sdk.calls == []


def test_motion_failure_still_attempts_disable(monkeypatch):
    sdk = FakeSDK()
    sdk.fail_move = True
    monkeypatch.setattr(zero_home, '_load_press_limits', lambda: LIMITS)
    monkeypatch.setattr(zero_home, 'TARGET_TOLERANCE_RAD', 0.001)
    with pytest.raises(zero_home.ProbeRejected):
        zero_home.run(sdk, execute=True)
    assert ('disable', 1) in sdk.calls
    assert sdk.flags == [False] * 6
    assert sdk.ctrl_mode == 0


def test_waypoint_is_streamed_until_feedback_stays_at_target(monkeypatch):
    sdk = FakeSDK()
    sdk.ctrl_mode = 1
    sdk.mode_feed = 1
    sdk.flags[0] = True
    target = (0.04, 0.0, 0.0, 0.0, 0.0)
    original_joint_ctrl = sdk.JointCtrl
    received = 0

    def delayed_joint_ctrl(*values):
        nonlocal received
        received += 1
        if received < 3:
            sdk.calls.append(('joint', values))
        else:
            original_joint_ctrl(*values)

    monkeypatch.setattr(sdk, 'JointCtrl', delayed_joint_ctrl)
    monkeypatch.setattr(zero_home, 'STREAM_PERIOD_SEC', 0.0)
    sent = zero_home._stream_target(sdk, target, 0, (1,), timeout_sec=0.2)
    assert sent >= 5
    assert received == sent


def test_recovery_restores_standby_only_when_all_motors_disabled():
    sdk = FakeSDK()
    sdk.ctrl_mode = 1
    sdk.mode_feed = 1
    zero_home.recover_standby(sdk)
    assert sdk.ctrl_mode == 0
    assert all(call[0] != 'joint' for call in sdk.calls)

    sdk.ctrl_mode = 1
    sdk.flags[0] = True
    sdk.calls.clear()
    with pytest.raises(zero_home.ProbeRejected, match=r'motor\(s\) enabled'):
        zero_home.recover_standby(sdk)
    assert sdk.calls == []


def test_default_execution_enables_only_j1(monkeypatch):
    sdk = FakeSDK()
    monkeypatch.setattr(zero_home, '_load_press_limits', lambda: LIMITS)
    zero_home.run(sdk, execute=True)
    assert [call for call in sdk.calls if call[0] == 'enable'] == [('enable', 1)]
    assert [call for call in sdk.calls if call[0] == 'disable'] == [('disable', 1)]


def test_cli_default_is_read_only(monkeypatch):
    sdk = FakeSDK()
    monkeypatch.setattr(zero_home, '_load_press_limits', lambda: LIMITS)
    monkeypatch.setitem(sys.modules, 'piper_sdk', SimpleNamespace(
        C_PiperInterface=lambda *, can_name: sdk))
    monkeypatch.setattr(sys, 'argv', ['zero_home'])
    assert zero_home.main() == 0
    assert sdk.calls == [('connect', False), ('disconnect',)]


def test_cli_recovery_only_sends_standby_mode(monkeypatch):
    sdk = FakeSDK()
    sdk.ctrl_mode = 1
    sdk.mode_feed = 1
    monkeypatch.setitem(sys.modules, 'piper_sdk', SimpleNamespace(
        C_PiperInterface=lambda *, can_name: sdk))
    monkeypatch.setattr(sys, 'argv', ['zero_home', '--recover-standby'])
    assert zero_home.main() == 0
    assert sdk.calls == [
        ('connect', False),
        ('mode', (0x00, 0x01, zero_home.SPEED_PERCENT, 0x00)),
        ('disconnect',),
    ]


def test_prior_mit_mode_blocks_all_commands(monkeypatch):
    sdk = FakeSDK()
    sdk.mode_feed = 4
    monkeypatch.setattr(zero_home, '_load_press_limits', lambda: LIMITS)
    with pytest.raises(zero_home.ProbeRejected, match='MIT'):
        zero_home.run(sdk, execute=True)
    assert sdk.calls == []
