# Piper press hardware commissioning

The feedback bridge is the first hardware integration gate. It reads Piper SDK CAN
feedback and publishes **J1–J5 positions only** on `hardware_joint_states`.
It never enables motors, subscribes to commands, sends trajectories, or runs
the simulation press task. A separate, explicitly armed single-motor
`enable_probe` is described below. J6 remains present in the physical electronics,
but its motor angle is not a press-tool degree of freedom because the tool is
mounted to the J6 stator.

The node calls `ConnectPort(piper_init=False)` to skip SDK initialization
queries. It publishes only when the SDK is healthy and joint feedback has a
fresh timestamp and plausible angles. Velocity and effort are left empty until
their units have been checked. Publication stops on missing/stale/invalid
feedback; no automatic hardware reconnect or command is attempted.

After the host `can0` is UP at 1 Mbit/s and `candump` shows current feedback,
build inside the Humble container:

```bash
cd /ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select piper_press_hardware
source /ws/install/setup.bash
ros2 launch piper_press_hardware hardware_feedback.launch.py
```

In a second container terminal, source the same ROS setups and inspect. The
explicit type, QoS and `--no-daemon` avoid a stale ROS CLI discovery daemon:

```bash
timeout 10s ros2 topic echo --no-daemon /hardware_joint_states sensor_msgs/msg/JointState --once --qos-reliability reliable
```

The launch deliberately does **not** publish to `/joint_states` yet, so it
cannot silently mix hardware feedback with an active Gazebo simulation.

## Read-only continuity check

With the robot stationary and the arm **not enabled**, this separate program
listens to the raw SocketCAN socket for Piper joint feedback IDs `0x2A3`–
`0x2A5`. It never sends a CAN frame. Run it in a container terminal while
the joint reader is running or by itself:

```bash
ros2 run piper_press_hardware can_gap_monitor --duration-sec 1800
```

It reports feedback counts every 10 seconds and logs `GAP_START`/`RECOVERED`
with wall-clock times. Exit code 1 means at least one interruption of 1 second
or more; exit code 2 means the CAN socket could not be opened. Ctrl+C prints
an `INTERRUPTED` summary and exits 130, rather than claiming that the full
requested duration passed. A clean 30-minute
run is useful diagnostic evidence, **not** authorization to move the arm. If
it stops receiving, capture the host kernel log around the printed time and
test another known-good USB port/cable or adapter with the electrical owner.

## Static robot-model check in RViz

Start the feedback reader first. Stop Gazebo, MoveIt, other robot-state
publishers and any controllers, then launch the viewer in another terminal:

```bash
ros2 launch piper_press_hardware hardware_model_view.launch.py
```

This viewer starts only `robot_state_publisher` and RViz. It expands the
five-axis press Xacro, remaps the publisher's `/joint_states` input to
`/hardware_joint_states`, and shows the model in the `world` frame. It does
not start `joint_state_publisher_gui`, MoveIt, Gazebo or hardware control.
Compare the displayed *stationary* pose, J1–J5 signs and `press_tcp` location
against the real arm; no calibration numbers should be inferred from one
unmeasured pose. RViz may retain the last image when feedback stops, so use
the reader health log and CAN monitor to determine whether the pose is live.

Do not start the stock Piper control launch, ros2_control hardware execution,
or `press_button_demo` against the physical arm at this stage.

## Execution preparation

`hardware_preflight` opens the CAN receive path with
`ConnectPort(piper_init=False)`, reads one fresh J1–J5 sample and arm status,
expands the **press-tool** Xacro, and compares the measured joint positions
against its strict URDF limits. It does not enable the arm or transmit motion.
Stop the feedback reader before this one-shot check so there is one SDK reader:

```bash
ros2 run piper_press_hardware hardware_preflight --can-port can0
```

A `FAIL joint limit` report is a model/zero-offset discrepancy to investigate,
not permission to widen a limit. Previously observed feedback put J2 slightly
below its URDF lower bound and J3 slightly above its upper bound. Confirm the
encoder zero, joint sign, manufacturer limits, and mechanical safe range with
the controls/mechanical owners before commanding motion. An enable-only probe
does not clear this trajectory gate. A passing preflight
is **not** a movement authorization.

### Single-motor enable-only commissioning probe

`enable_probe` selects exactly one motor in J1–J5; J6 and the SDK's all-motor
selector are rejected. By default it reads fresh joint, arm and motor-status
feedback, checks that all six motors are initially disabled, and sends **no**
command. An explicit `--execute` sends one selected `EnableArm`, waits for its
enable feedback, observes the pose briefly, then sends `DisableArm` for that
motor and checks disabled feedback. It never calls `ModeCtrl`, `JointCtrl`, or
any gripper command. A 0.02 rad shift gate is a diagnostic trigger, not a
guarantee that a shift cannot occur.

For J2 or J3, the probe also requires **both** J2 and J3 feedback to be at
least 0.05 rad inside their press-tool URDF limits before sending an enable
command. This is a commissioning clearance gate, not a zero-offset
calibration. The presently observed folded, disabled pose lies outside that
gate; do not widen limits or apply a guessed 2-degree offset to bypass it.

For any executed test, clear the workspace, keep a physical support/person in
place against arm drop, and keep an independent hardware stop immediately
reachable. CAN loss may prevent the best-effort disable from arriving. The
software probe, the OEM GUI stop, and unplugging USB are **not** substitutes
for an independent stop. Do not run the stock Piper controller, MoveIt,
Gazebo, the feedback reader, or another SDK control process in parallel.

Start with a **dry run for J1** in the container after building this package:

```bash
ros2 run piper_press_hardware enable_probe --motor 1
```

Only after its output shows fresh all-disabled feedback, the physical safety
arrangements are confirmed, and a human explicitly chooses to proceed, the
same isolated test can be run with `--execute`. It may hold or shift the arm
when enabling. Do not proceed to J2–J5 or any position/trajectory command
merely because J1 passes. A failed probe requires checking the arm in person;
do not assume its final motor state from a failed CAN command.

### First position-command and zero-directed motion test

`zero_home` is a **separate commissioning tool**, not the hardware trajectory
controller. Its default invocation is receive-only and prints the current
joint angles, required initial in-limit setpoints, the proposed goal and the
number of small steps:

```bash
ros2 run piper_press_hardware zero_home
```

With an explicit execute/clearance acknowledgement, the default active mode
enables **J1 only** and commands at most 1 degree toward J1 zero at a 5% SDK
speed setting. It checks CAN joint-mode feedback and every waypoint, then
attempts to disable J1. Each target is refreshed at 50 Hz until three
consecutive fresh feedback samples are within tolerance; a 3-second waypoint
timeout is a failure, not evidence of arrival. Do not execute until the preview and the physical
path/clearance have been reviewed. The separate `--full-home` option previews
or, with the same execute acknowledgement, attempts a bounded all-J1–J5
return to zero. Full-home execution rejects any starting joint outside its
URDF limits, including the previously observed J2/J3 droop, so the
default J1-only test must precede it. Do not infer collision-free motion from
the URDF joint-limit checks alone.

```bash
# First real position command, only after clearing the workspace and reviewing preview:
ros2 run piper_press_hardware zero_home --execute --confirm-clear-and-supported

# Separate preview of the larger motion; not an instruction to execute it yet:
ros2 run piper_press_hardware zero_home --full-home
```

After a failed earlier run leaves the controller in `CAN_CTRL/MOVE_J`, recover
STANDBY with the separate explicit command below. It refuses to transmit if
any of the six motors reports enabled; it sends no joint target or enable:

```bash
ros2 run piper_press_hardware zero_home --recover-standby
```

The SDK `JointCtrl` packet has six fields. This tool always holds J6 at its
current feedback angle in that packet but **never enables J6**. It sends no
gripper command. It rejects motion more than 0.40 rad from zero, starts only
from fresh healthy STANDBY feedback with all six motors disabled, and rejects
unexpected teach/MIT mode. On completion or failure it attempts to disable
only the motors it enabled, verifies all-six disabled feedback, then restores
STANDBY. CAN failure can defeat these attempts. Maintain
physical support and an independent stop throughout. Because motors are
disabled at the end, this test does **not** leave the arm holding zero and is
not a way to hand control directly to the press algorithm.

`trajectory_guard.py` is a pure-Python validation core for a future execution
adapter. It requires fresh feedback, the exact J1–J5 command order, points
within URDF position limits, a matched start state, increasing timestamps,
and explicitly supplied step, average-speed, and duration caps. It returns
only a validation summary or a structured rejection; there is no ROS command
subscriber, ros2_control plugin, SDK motion call, or CAN transmit path yet.
Average-speed checks do not replace controller velocity/acceleration limits,
physical watchdogs, cancellation characterization, or emergency stopping.
The upstream `piper_ctrl_single_node` enables all six motors and sends a
six-joint `JointCtrl` packet, so it must not be reused as the press hardware
adapter without a separate J6/safety design review.

Before running the press algorithm: establish calibrated J1–J5 joint signs
and offsets, verify the real press TCP and J6 stator mount, implement a
continuous J1–J5 hardware execution adapter with explicit enable/stop gates,
and validate emergency-stop behavior. The commissioning probes are not that
adapter.
