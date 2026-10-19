#!/usr/bin/env python3
"""Static contract checks for the semantic hardware bringup wrapper."""

import unittest
from pathlib import Path


LAUNCH = (
    Path(__file__).parents[1]
    / "launch"
    / "press_button_semantic_hardware.launch.py"
)


class SemanticHardwareLaunchTest(unittest.TestCase):
    def test_wrapper_owns_semantic_adapter_not_camera_or_hardware_bringup(self):
        source = LAUNCH.read_text(encoding="utf-8")

        self.assertIn('executable="button_target_selector.py"', source)
        self.assertIn('"target_source": "topic"', source)
        self.assertIn('"target_topic": "/press/target_pose"', source)
        self.assertIn('"max_presses": "1"', source)
        self.assertIn('"refine_target_topic": "/press/refine_pose"', source)
        self.assertIn('DeclareLaunchArgument("target_semantic"', source)
        self.assertIn('press_button_hardware.launch.py', source)
        self.assertNotIn('piper_elevator_button_perception', source)
        self.assertNotIn('piper_real_bringup.launch.py', source)


if __name__ == "__main__":
    unittest.main()
