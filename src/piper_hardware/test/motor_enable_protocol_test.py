#!/usr/bin/env python3
"""Protocol regression test for Piper's all-motor enable selector."""

from pathlib import Path
import re
import unittest


class MotorEnableProtocolTest(unittest.TestCase):
    @staticmethod
    def _function_body(source: str, function_name: str) -> str:
        match = re.search(
            rf"bool PiperSystem::{function_name}\([^{{]+\) \{{"
            r"(.*?)(?=\n\}\n\n(?:bool|void|hardware_interface::)|\Z)",
            source,
            re.DOTALL,
        )
        if match is None:
            raise AssertionError(f"could not locate PiperSystem::{function_name}")
        return match.group(1)

    def test_vendor_enable_handshake_uses_selector_seven(self) -> None:
        source = (
            Path(__file__).resolve().parents[1] / "src" / "piper_system.cpp"
        ).read_text(encoding="utf-8")
        match = re.search(
            r"constexpr uint8_t kMotorAllJoints\s*=\s*(0x[0-9A-Fa-f]+)", source
        )
        self.assertIsNotNone(match, "Piper all-motor selector must be explicit")
        self.assertEqual(
            match.group(1).lower(),
            "0x07",
            "the installed Piper V2 SDK's EnablePiper handshake uses selector 0x07",
        )

    def test_control_frames_use_the_protocol_eight_byte_dlc(self) -> None:
        source = (
            Path(__file__).resolve().parents[1] / "src" / "piper_system.cpp"
        ).read_text(encoding="utf-8")
        for function_name in ("setMotorsEnabled", "sendMotionMode"):
            body = self._function_body(source, function_name)
            self.assertRegex(
                body,
                r"frame\.length\s*=\s*8;",
                f"PiperSystem::{function_name} must zero-pad its CAN frame to DLC 8",
            )

    def test_vendor_enable_handshake_resumes_and_confirms_feedback(self) -> None:
        source = (
            Path(__file__).resolve().parents[1] / "src" / "piper_system.cpp"
        ).read_text(encoding="utf-8")
        resume = self._function_body(source, "sendEmergencyStopResume")
        self.assertRegex(resume, r"frame\.length\s*=\s*8;")
        self.assertRegex(resume, r"frame\.data\[0\]\s*=\s*kEmergencyStopResume;")

        enable = self._function_body(source, "enableMotorsAndConfirm")
        self.assertIn("setMotorsEnabled(true, error)", enable)
        self.assertIn("motorsAllEnabled()", enable)
        self.assertRegex(
            enable,
            r"(?s)while \(.*?setMotorsEnabled\(true, error\).*?"
            r"pollFeedback\(\).*?\}\s*if \(motorsAllEnabled\(\)\)",
            "enable requests must cover the whole handshake window before confirmation",
        )

    def test_enable_handshake_never_exposes_the_firmware_zero_target(self) -> None:
        """The measured pose must be streamed before and throughout enabling."""
        source = (
            Path(__file__).resolve().parents[1] / "src" / "piper_system.cpp"
        ).read_text(encoding="utf-8")

        enable = self._function_body(source, "enableMotorsAndConfirm")
        enable_request = enable.index("setMotorsEnabled(true, error)")
        restore_mode = enable.index("sendMotionMode(error)")
        restore_hold = enable.index("sendJointCommand(hold, error)")
        feedback_wait = enable.index("sleep_for")
        self.assertLess(
            enable_request,
            restore_mode,
            "each enable request must be followed by restoring MOVE_J mode",
        )
        self.assertLess(
            restore_mode,
            restore_hold,
            "the measured-pose hold must follow the restored MOVE_J mode",
        )
        self.assertLess(
            restore_hold,
            feedback_wait,
            "never leave the firmware zero target active during the feedback wait",
        )

        activation = re.search(
            r"PiperSystem::on_activate\([^\{]+\) \{(.*?)\n\}",
            source,
            re.DOTALL,
        )
        self.assertIsNotNone(activation)
        body = activation.group(1)
        self.assertLess(
            body.index("sendJointCommand(hold, &error)"),
            body.index("enableMotorsAndConfirm(hold, &error)"),
            "activation must preload the measured-pose hold before enabling motors",
        )


if __name__ == "__main__":
    unittest.main()
