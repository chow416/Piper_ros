"""No physical CAN is used by these enable-probe tests."""

from types import SimpleNamespace
import sys
import time

import pytest

from piper_press_hardware import enable_probe
from piper_press_hardware.enable_probe import ProbeRejected, run
from piper_press_hardware.model_limits import JointLimit


class FakeSDK:
    def __init__(self):
        self.flags = [False] * 6
        self.positions = [0] * 6
        self.calls = []
        self.ok = True
        self.error_code = 0
        self.mode = 0
        self.missing_motor = None

    def isOk(self):
        return self.ok

    def GetArmJointMsgs(self):
        return SimpleNamespace(
            time_stamp=time.time(),
            joint_state=SimpleNamespace(**{
                f'joint_{index}': position
                for index, position in enumerate(self.positions, start=1)
            }),
        )

    def GetArmStatus(self):
        return SimpleNamespace(
            time_stamp=time.time(),
            arm_status=SimpleNamespace(
                err_code=self.error_code,
                arm_status=0,
                ctrl_mode=self.mode,
                teach_status=0,
            ),
        )

    def GetArmLowSpdInfoMsgs(self):
        return SimpleNamespace(
            time_stamp=time.time(),
            **{
                f'motor_{index}': SimpleNamespace(
                    can_id=0 if self.missing_motor == index else 0x260 + index,
                    foc_status=SimpleNamespace(driver_enable_status=flag),
                )
                for index, flag in enumerate(self.flags, start=1)
            },
        )

    def EnableArm(self, *, motor_num):
        self.calls.append(('enable', motor_num))
        self.flags[motor_num - 1] = True

    def DisableArm(self, *, motor_num):
        self.calls.append(('disable', motor_num))
        self.flags[motor_num - 1] = False


def test_default_dry_run_makes_no_command(capsys):
    sdk = FakeSDK()
    run(sdk, 1)
    assert sdk.calls == []
    assert 'DRY_RUN PASS' in capsys.readouterr().out


@pytest.mark.parametrize('motor', [6, 7, 255, 0])
def test_j6_and_all_motor_selectors_are_rejected(motor):
    sdk = FakeSDK()
    with pytest.raises(ValueError, match='motor must be'):
        run(sdk, motor, execute=True)
    assert sdk.calls == []


def test_one_motor_enable_then_disable(monkeypatch):
    sdk = FakeSDK()
    monkeypatch.setattr('piper_press_hardware.enable_probe.time.sleep', lambda _: None)
    run(sdk, 1, execute=True)
    assert sdk.calls == [('enable', 1), ('disable', 1)]
    assert sdk.flags == [False] * 6


def test_preexisting_enable_fails_closed():
    sdk = FakeSDK()
    sdk.flags[5] = True
    with pytest.raises(ProbeRejected, match='already enabled'):
        run(sdk, 1, execute=True)
    assert sdk.calls == []


def test_missing_motor_status_is_not_mistaken_for_disabled():
    sdk = FakeSDK()
    sdk.missing_motor = 6
    with pytest.raises(ProbeRejected, match='J6 has no valid'):
        run(sdk, 1, execute=True)
    assert sdk.calls == []


def test_gravity_joint_near_limit_cannot_be_enabled(monkeypatch):
    sdk = FakeSDK()
    sdk.positions[1] = -2187
    sdk.positions[2] = 1618
    limits = (
        JointLimit(-2.618, 2.618),
        JointLimit(0.0, 3.14),
        JointLimit(-2.967, 0.0),
        JointLimit(-1.745, 1.745),
        JointLimit(-1.22, 1.22),
    )
    monkeypatch.setattr('piper_press_hardware.hardware_preflight._load_press_limits',
                        lambda: limits)
    with pytest.raises(ProbeRejected, match='J2='):
        run(sdk, 2, execute=True)
    assert sdk.calls == []


@pytest.mark.parametrize('property_name,value', [
    ('ok', False),
    ('error_code', 7),
    ('mode', 2),
])
def test_bad_feedback_or_teach_mode_fails_closed(property_name, value):
    sdk = FakeSDK()
    setattr(sdk, property_name, value)
    with pytest.raises(ProbeRejected):
        run(sdk, 1, execute=True)
    assert sdk.calls == []


def test_excessive_shift_still_disables(monkeypatch):
    sdk = FakeSDK()
    original_enable = sdk.EnableArm

    def shifted_enable(*, motor_num):
        original_enable(motor_num=motor_num)
        sdk.positions[0] = 3000  # 3 degrees, larger than the 0.02 rad gate.

    sdk.EnableArm = shifted_enable
    monkeypatch.setattr('piper_press_hardware.enable_probe.time.sleep', lambda _: None)
    with pytest.raises(ProbeRejected, match='shifted'):
        run(sdk, 1, execute=True)
    assert sdk.calls == [('enable', 1), ('disable', 1)]


def test_cli_defaults_to_receive_only(monkeypatch):
    sdk = FakeSDK()

    class Factory:
        def __new__(cls, *, can_name):
            assert can_name == 'can0'
            return sdk

    sdk.ConnectPort = lambda *, piper_init: sdk.calls.append(
        ('connect', piper_init))
    sdk.DisconnectPort = lambda: sdk.calls.append(('disconnect',))
    monkeypatch.setitem(sys.modules, 'piper_sdk',
                        SimpleNamespace(C_PiperInterface=Factory))
    monkeypatch.setattr(sys, 'argv', ['enable_probe', '--motor', '1'])
    assert enable_probe.main() == 0
    assert sdk.calls == [('connect', False), ('disconnect',)]
