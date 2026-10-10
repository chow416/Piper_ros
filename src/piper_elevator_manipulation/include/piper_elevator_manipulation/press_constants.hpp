#pragma once

#include <array>

namespace piper_elevator_manipulation {

// Names shared by every module that talks to MoveIt or MTC about this arm.
// They were file-local constants in press_button_demo.cpp before the split.
inline constexpr char kArmGroup[] = "arm";
inline constexpr char kEndEffectorLink[] = "press_tcp";
inline constexpr char kPlanningFrame[] = "world";
inline constexpr std::array<const char*, 5> kArmJoints{
    "joint1", "joint2", "joint3", "joint4", "joint5"};

}  // namespace piper_elevator_manipulation
