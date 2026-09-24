# piper_hardware

`ros2_control` hardware interface that drives a **physical** Piper arm over
SocketCAN. Its purpose is to make the real robot a drop-in replacement for the
Gazebo simulation: the same controllers, the same `piper_press_moveit`
configuration and the same `press_button_demo` task run on both, with only the
hardware plugin changing.

## What it does

| Direction | CAN ID | Content |
|---|---|---|
| read | `0x2A5` | joint1, joint2 (int32 little-endian, 0.001°) |
| read | `0x2A6` | joint3, joint4 |
| read | `0x2A7` | joint5, joint6 |
| read | `0x2A1`, `0x2A8` | liveness only (status, gripper) |
| write | `0x471` | motor enable / disable (motor 7 = all joints) |
| write | `0x151` | control mode, MOVE J, speed ratio, position-velocity mode |
| write | `0x155` | joint1, joint2 command |
| write | `0x156` | joint3, joint4 command |
| write | `0x157` | joint5, joint6 command |

The mapping was verified against `piper_sdk` and cross-checked against a live
arm: the values in `0x2A3`/`0x2A4` match the end-effector pose feedback, and
`0x2A5`/`0x2A6` match the joint angle feedback.

## Safety behaviour

These are deliberate and should not be relaxed without thought:

- **Nothing moves by default.** `enable_motors_on_activate` defaults to `false`,
  so the first bring-up exercises the whole chain with the arm limp.
- **The command buffer is seeded from measured feedback.** Activation blocks
  until every joint angle has been received, then initialises both the state and
  the command to the *measured* pose. Without this the first controller cycle
  would command a zero-initialised buffer, which means "drive every joint to
  zero".
- **Activation fails loudly** if the arm does not answer within three seconds,
  rather than starting up blind.
- **Feedback watchdog.** If no arm feedback arrives for `feedback_timeout`
  seconds, `read()` returns an error so the controller manager deactivates the
  controllers instead of continuing to command motion.
- **Undefined joints are held, not released.** joint6 is not part of the
  five-axis press description, so the driver keeps commanding its last measured
  angle instead of leaving it free.
- **Protocol limit clamp.** Commands are clamped to the vendor limits as a last
  resort, independent of the URDF.
- **Non-finite commands** fall back to the measured angle instead of being sent.

## Usage

Confirm that `can0` is up at 1 Mbps first:

```bash
ip -details link show can0
```

### 1. Dry run: software only, arm limp

```bash
ros2 launch piper_hardware piper_real_bringup.launch.py
```

Then check that the arm is readable:

```bash
ros2 control list_hardware_interfaces
ros2 topic echo /joint_states --once
```

`/joint_states` must report the arm's real pose. If it does, the CAN layer,
the protocol decoding and the interface export are all correct.

### 2. Verify motion with a very small command

With the arm limp, a joint trajectory command produces no motion but still
exercises the full command path. To actually move, enable the motors:

```bash
ros2 launch piper_hardware piper_real_bringup.launch.py \
  enable_motors:=true command_speed_ratio:=20
```

Keep the emergency stop within reach. `command_speed_ratio` scales the arm-side
speed and is the first thing to reduce when unsure.

## Parameters

Set as `<param>` entries inside the `<hardware>` block of
`urdf/piper_real.urdf.xacro`.

| Parameter | Default | Meaning |
|---|---|---|
| `can_interface` | `can0` | SocketCAN interface |
| `enable_motors_on_activate` | `false` | Energise the motors when the interface activates |
| `disable_motors_on_deactivate` | `true` | Release the motors when it deactivates |
| `feedback_timeout` | `0.5` | Seconds without feedback before a hardware failure is reported |
| `command_speed_ratio` | `100` | Arm-side speed percentage (clamped to 1-100) |
| `clamp_to_protocol_limits` | `true` | Clamp commands to the vendor joint limits |

## Not implemented yet

- **Force or contact detection.** The press phase stops at a configured travel,
  not on contact. On hardware this is the main remaining risk.
- **Status and error decoding.** `0x2A1` is used as a liveness heartbeat only;
  its error code field is not interpreted yet.
- **Command rate limiting.** Every cycle sends three CAN frames. At 200 Hz that
  is roughly 8% bus load on top of the arm's ~40% feedback traffic; raising the
  controller update rate would need a divider.
