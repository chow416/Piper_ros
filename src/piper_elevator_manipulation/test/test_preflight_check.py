#!/usr/bin/env python3
"""Regression tests for the hardware-health part of preflight_check."""

import importlib.util
import unittest
from pathlib import Path
from unittest import mock


SCRIPT = Path(__file__).parents[1] / "scripts" / "preflight_check.py"
SPEC = importlib.util.spec_from_file_location("preflight_check", SCRIPT)
PREFLIGHT = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(PREFLIGHT)


class PreflightHardwareHealthTest(unittest.TestCase):
    """Catch false-ready states that let a live run reach a dead driver."""

    def test_can_traffic_is_unhealthy_when_rx_counter_does_not_advance(self):
        with mock.patch.object(
                PREFLIGHT, "read_can_rx_packets", side_effect=[17997826, 17997826]), \
                mock.patch.object(PREFLIGHT.time, "sleep"):
            live, before, after = PREFLIGHT.can_receives_frames(
                "can0", sample_seconds=0.25)

        self.assertFalse(live)
        self.assertEqual((before, after), (17997826, 17997826))

    def test_can_traffic_is_healthy_when_rx_counter_advances(self):
        with mock.patch.object(
                PREFLIGHT, "read_can_rx_packets", side_effect=[100, 140]), \
                mock.patch.object(PREFLIGHT.time, "sleep"):
            live, before, after = PREFLIGHT.can_receives_frames(
                "can0", sample_seconds=0.25)

        self.assertTrue(live)
        self.assertEqual((before, after), (100, 140))

    def test_hardware_state_parser_detects_unconfigured_piper_system(self):
        output = """Hardware Component 1
\tname: \x1b[93mPiperSystem\x1b[0m
\ttype: system
\tstate: id=1 label=\x1b[93munconfigured\x1b[0m
"""

        self.assertEqual(
            PREFLIGHT.parse_hardware_component_state(output, "PiperSystem"),
            "unconfigured")

    def test_hardware_state_parser_detects_active_piper_system(self):
        output = """Hardware Component 1
\tname: \x1b[92mPiperSystem\x1b[0m
\ttype: system
\tstate: id=3 label=\x1b[92mactive\x1b[0m
"""

        self.assertEqual(
            PREFLIGHT.parse_hardware_component_state(output, "PiperSystem"),
            "active")


if __name__ == "__main__":
    unittest.main()
