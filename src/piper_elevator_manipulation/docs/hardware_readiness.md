# Hardware readiness gate for the Piper press task

The simulation launch is unchanged and still requires `use_sim_time=true` plus an
explicitly enabled virtual contact plane. A
successful Gazebo run is not permission or evidence to command the real arm.

A separate hardware profile now exists
(`launch/press_button_hardware.launch.py`), gated behind `execute:=true` *and*
`hardware_enable:=true` with `use_sim_time:=false`, and offering
`execution_mode=hardware_fixed_travel` for the case where no contact sensor is
available yet. Keep `execute:=false` until every item below is verified with the
mechanical, electrical, and controls owners.

1. Confirm the as-built J6 stator-mounted tool geometry, joint signs/limits,
   `press_tcp` position and +Z axis, and mounting transform against the real
   mechanism. Compare FK to measured tool positions at several safe poses.
2. Bring up a hardware `ros2_control` J1–J5 controller separately. Verify
   feedback freshness, trajectory tracking, command cancellation/hold,
   watchdog behavior, and that no gripper/J6 command is issued.
   **DONE** with `piper_hardware` (`piper_hardware/PiperSystem`): single-joint
   moves and joint-trajectory goals track to within 0.05 deg, the feedback
   watchdog is implemented, and J6 is held rather than commanded.
3. Connect a real force, motor-current, switch, or button-displacement contact
   source through a timestamped adapter. Validate stable-contact thresholds,
   false positives/negatives, unplugged-sensor behavior, and sensor latency.
   The `/press/contact` virtual publisher must not run on hardware.
4. Provide a stamped target interface and transform the target from its
   declared frame into the current planning frame. Reject stale/unknown frames
   and targets outside a calibrated workspace. The current launch parameters
   assume a fixed `world` frame and are not this interface.
   **PARTLY DONE**: `target_source:=topic` subscribes to a stamped
   `geometry_msgs/PoseStamped` and rejects empty frames, frames other than
   `world`/`base_link`, zero or stale stamps, non-finite numbers, unset
   quaternions and points outside the workspace box. See
   `docs/perception_interface.md`. **Missing**: the actual TF lookup, so a
   target declared in a camera frame is still refused rather than transformed.
   `world` and `base_link` are the same frame (fixed joint, identity), which is
   why both can be accepted without a transform.
5. Measure worst-case stop latency and TCP overshoot at reduced speed, with
   independent protective limits and an accessible emergency stop. A software
   4 mm cancellation threshold is not a hard physical stop.
6. Validate collision geometry and the real environment, including the
   elevator panel and mobile-base pose. The present Gazebo world has no
   physical button or panel, and its contact event is geometric simulation.
7. Exercise dry-run, single-step free motion, approach, press timeout, missing
   contact, controller fault, emergency stop, retreat, and return under a
   supervised test protocol. Only then create a separate hardware launch
   profile with an explicit enable gate.
   **PROFILE CREATED**, exercise still outstanding: dry-run, free motion,
   approach, fixed-travel press, retreat and return have each been run on the
   real arm. Press timeout, controller fault and emergency stop have not.

## Home pose (`go_home.py`)

There is no homing switch on this arm, so "home" is a software pose, not a
measured one. It serves three purposes: give the operator a known repeatable
staging pose, move the arm out of the folded `joint2 ~ 0 / joint3 ~ 0`
configuration where the 5-DOF IK loses rank, and leave the joints well inside
the firmware soft limits so the servos hold rather than push.

The pose lives in `config/home_pose.yaml` and is validated at runtime against
the protocol joint limits before anything is sent. `scripts/go_home.py` reads
it, takes the measured pose as the trajectory start point, and moves over a
fixed duration. Verified on the real arm: all five joints landed within
**0.082 deg** of the configured values.

Expected command sequence on hardware:

```
1. ros2 launch piper_hardware piper_real_bringup.launch.py \
       enable_motors:=true command_speed_ratio:=20
2. ros2 run piper_elevator_manipulation go_home.py --dry-run   # optional
3. ros2 run piper_elevator_manipulation go_home.py
4. ros2 launch piper_elevator_manipulation press_button_hardware.launch.py \
       execute:=true hardware_enable:=true <target args>
```

Step 1 must come first: the arm is limp until the motors are enabled, and a
limp arm rests against its mechanical stops slightly outside the firmware
limits. `go_home.py` also requires the `arm_controller` action server, which
the bringup provides.

> Measured behaviour worth carrying into the first real-button test: the
> controller leaves about **2 mm of axial tracking error** at the end of the
> approach, so a 3 mm press produces roughly 1 mm of net travel past the target.
> Press depth must budget for that, not assume the commanded value.
>
> Also note that the task IK searches the whole joint space on every call, so an
> unreachable target cannot be rescued by changing its seed. On hardware the
> effective remedy is to move the arm out of a degenerate parked pose first, and
> an arm parked with the motors disabled sits against its mechanical stops a few
> hundredths of a radian outside the firmware limits.

Until these gates are closed, do **not** remove the simulation execution guard
from `press_button_demo.cpp` or redirect this demo to the hardware controller.
