from types import SimpleNamespace
import sys
import time

import pytest

from piper_press_hardware import hardware_preflight
from piper_press_hardware.model_limits import JointLimit


LIMITS = (
    JointLimit(-2.618, 2.618),
    JointLimit(0.0, 3.14),
    JointLimit(-2.967, 0.0),
    JointLimit(-1.745, 1.745),
    JointLimit(-1.22, 1.22),
)


@pytest.mark.parametrize('raw_positions,expected_result', [
    ((0, -2187, 1618, 0, 0), 1),
    ((0, 1000, -1000, 0, 0), 0),
])
def test_preflight_only_reads_and_detects_limit_mismatch(
        monkeypatch, capsys, raw_positions, expected_result):
    calls = []

    class FakeSDK:
        def __init__(self, can_name):
            assert can_name == 'can0'

        def ConnectPort(self, *, piper_init):
            calls.append(('connect', piper_init))

        def isOk(self):
            return True

        def GetArmJointMsgs(self):
            state = SimpleNamespace(**{
                f'joint_{index}': position
                for index, position in enumerate(raw_positions, start=1)
            })
            return SimpleNamespace(time_stamp=time.time(), joint_state=state)

        def GetArmStatus(self):
            return SimpleNamespace(
                time_stamp=time.time(),
                arm_status=SimpleNamespace(err_code=0),
            )

        def DisconnectPort(self):
            calls.append(('disconnect',))

    monkeypatch.setattr(hardware_preflight, '_load_press_limits', lambda: LIMITS)
    monkeypatch.setitem(sys.modules, 'piper_sdk',
                        SimpleNamespace(C_PiperInterface=FakeSDK))
    assert hardware_preflight.run('can0', 0.5, 0.25) == expected_result
    assert calls == [('connect', False), ('disconnect',)]
    output = capsys.readouterr().out
    assert 'no CAN commands' in output
    assert ('FAIL joint limit' in output) == (expected_result == 1)
