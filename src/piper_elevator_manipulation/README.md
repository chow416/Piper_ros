# Piper elevator manipulation demo

`press_button_demo` is a five-axis MoveIt Task Constructor (MTC)
planning/control simulation baseline. It runs the following sequence from
the live Gazebo joint state:

1. capture `joint1` through `joint5` as the return state;
2. solve a five-dimensional pre-press target; follow the prerecorded free-motion
   joint waypoints if every segment is collision-free, otherwise use OMPL to
   reach the same pre-press joint state;
3. approach the target on a sampled, collision-checked Cartesian line along
   the panel normal;
4. send a separately speed-limited press trajectory, cancel it when stable
   contact is reported, and treat its configured depth as a *maximum travel
   limit*, not proof of success;
5. replan a panel-normal retreat from the **actual stopped joint state**, then
   return to the captured start state. Missing contact is a failed task even
   if the arm returns safely.

The target can come from launch parameters (a fixed `button_x/y/z` point) or
from a topic, selected with `target_source`. The topic form is the interface the
perception layer uses; its contract, validation rules and a copy-pasteable
publisher example are in `docs/perception_interface.md`.

The target is supplied explicitly in `world`. Its
task contract is a point plus a press direction, not an arbitrary 6D pose:
rotation around the cylindrical press rod is intentionally free because J6 is
not actuated. `FiveDofIkSolver` numerically minimizes TCP position error and
press-axis direction error, then MTC performs collision checking and trajectory
planning. The package has no dependency on the perception layer; it only sees
the topic contract described above.

Run a dry plan first:

```bash
ros2 launch piper_elevator_manipulation press_button_demo.launch.py
```

Or drive it from the target topic, which is what the perception layer will do:

```bash
ros2 launch piper_elevator_manipulation press_button_demo.launch.py \
    target_source:=topic

# in another terminal, as many times as you like
ros2 run piper_elevator_manipulation send_press_target.py \
    --point 0.300,0.000,0.240 --direction 1,0,0
```

Then execute in Gazebo with the explicit simulation-only virtual contact source:

```bash
ros2 launch piper_elevator_manipulation press_button_demo.launch.py \
  execute:=true simulated_contact:=true
```

The default target is the virtual world-frame point `(0.300, 0.000, 0.240)`
with press direction `(+1, 0, 0)`, a 30 mm pre-press clearance, and 4 mm
maximum press travel. No physical panel or button is present in the default simulation
world. Override `button_x`, `button_y`, `button_z`, `press_direction_x`,
`press_direction_y`, `press_direction_z`, `approach_distance`, or
`press_depth` as needed. The launch starts the five-axis Gazebo model, the
`piper_press_moveit` configuration, RViz, and MTC's execution capability.

The virtual contact adapter publishes `/press/contact` when the modeled TCP
crosses a configurable plane, by default exactly at the supplied target.
This is a deterministic simulation signal, **not measured force or physical
contact**. For a no-contact safety test, set
`virtual_contact_offset_m:=0.009` with the default 4 mm travel cap: the task
must report `CONTACT_NOT_DETECTED` after retreating and returning.
`execute:=true` without `simulated_contact:=true` is rejected before any
controller command. The executable also requires `use_sim_time=true`; do not
use this launch file for hardware.

During pressing, a second monitor computes the TCP's *measured* travel from
fresh joint feedback and requests controller cancellation at `press_depth`,
even if the planned endpoint or controller tracking differs. Stale feedback
also stops the press. This cap is independent of the contact signal, but
physical motion can overshoot it during sensor/controller stop latency; that
latency has not been qualified on hardware.

## Running on the real arm

The step-by-step operating procedure for the real arm is in
`docs/run_press_task.md` (Chinese). The short version:

The hardware path replaces only the ros2_control plugin, so the planning stack
above is unchanged. The bringup must come first, because a limp arm rests
against mechanical stops that sit slightly outside the firmware soft limits and
nothing can be planned from that pose.

```bash
# 1. hardware bringup + MoveIt, motors enabled and holding
ros2 launch piper_hardware piper_real_bringup.launch.py \
    enable_motors:=true command_speed_ratio:=20

# 2. optional: check the home pose without sending anything
ros2 run piper_elevator_manipulation go_home.py --dry-run

# 3. move to the configured home/staging pose
ros2 run piper_elevator_manipulation go_home.py

# 4. plan and press (simulation guard replaced by the hardware gate)
ros2 launch piper_elevator_manipulation press_button_hardware.launch.py \
    execute:=true hardware_enable:=true button_x:=0.30 button_z:=0.24
```

`go_home.py` reads `config/home_pose.yaml`, validates the values against the
protocol joint limits, and refuses to send anything if a value is out of range.
Edit that file to change the pose. Useful flags: `--duration` (seconds, default
8.0), `--config`, `--controller`, `--dry-run`. Measured on the real arm, all
five joints land within 0.082 deg of the configured values.

The Cartesian line planner interpolates TCP position along the supplied
world-frame press direction, solves 5D IK for each sample, and checks press-axis
alignment, joint jumps, limits, and collisions. The press, approach, and
retreat speed scalings and Cartesian sampling tolerances are configured in
`config/free_motion_route.yaml`. Trajectory execution is separate from contact
monitoring; the contact signal can later be supplied by a validated real
sensor adapter without changing the geometric planner.

The default free-motion route is in `config/free_motion_route.yaml`. Each
waypoint records all five joint angles in radians, not a precomputed trajectory.
At runtime the planner interpolates between the current state, recorded
waypoints, and the computed pre-press state, checking joint limits and
collisions throughout. If any recorded segment fails, MTC tries the OMPL
sampling branch. The log line `free_motion.path_source=prerecorded` or
`free_motion.path_source=sampling` describes the branch in the *selected full
task solution*. A different route file can be selected with
`free_route_file:=/absolute/path/to/route.yaml`.

The task IK is seeded from the **last recorded waypoint**, not from the captured
start state. A five-axis arm asked for a position plus a press-axis direction has
a family of solutions, and the seed decides which branch is returned. Seeding
from the start state answers the wrong question: by the time the press begins the
arm has already driven the route, so the branch should be the one nearest the
route's end pose. The start-state seed is kept as a fallback and is used when the
route-seeded solve fails, so a route that cannot be solved for degrades instead
of failing the task. The log line `5D IK solved from seed '...'` names the seed
that produced the plan.

Note what this does **not** do: `FiveDofIkSolver` already restarts from the joint
range midpoint and 32 random configurations, so it searches the whole joint space
on every call. If a target is reported unreachable, no choice of seed will make it
reachable. On hardware the reliable remedy is to move the arm out of a degenerate
parked pose first, which is what the recorded route is for.

In Gazebo, joints initialized exactly at their limits may fluctuate slightly
beyond them, and a real arm parked with the motors disabled sits against the
mechanical stops, a few hundredths of a radian outside the firmware limits. The
recorded-route planner clamps only start-state noise up to
`start_bound_tolerance_rad` (default 0.005 rad, calibrated from the measured
hardware figure of about 0.0025 rad; the simulator only ever showed a few 1e-5).
Larger deviations reject the recorded route, and the sampler takes over. Set this
tolerance only after measuring the simulator or hardware feedback; the validator
rejects anything above 0.005 rad, so it cannot be widened into hiding a real
fault.

This is still an integration demo: the phase order is hard-coded in one
executable rather than exposed as cancellable task actions under a Behavior
Tree. Its contact event comes from the virtual simulation plane only.

The custom IK tolerances, numerical restart count, speed limits and provisional
tool dimensions remain development parameters. Before hardware motion,
integrate and validate a real contact sensor, emergency stop, joint/calibration
limits, target-frame transform and freshness checks, and fault recovery on the
actual arm. A successful virtual-plane simulation is not hardware qualification.
See [`docs/hardware_readiness.md`](docs/hardware_readiness.md) before attempting
real-arm integration.

## Simulated wrist RGB-D camera

The Gazebo robot has a wrist-mounted depth camera shared by the Gazebo and
MoveIt robot descriptions.  It publishes:

- `/wrist_camera/depth/image_raw` (RGB image)
- `/wrist_camera/depth/depth/image_raw` (depth image)
- `/wrist_camera/depth/points` (`sensor_msgs/PointCloud2`)
- `/wrist_camera/filtered_points` (MoveIt self-filtered point cloud)

MoveIt consumes the point cloud with `PointCloudOctomapUpdater` and maintains
a 3 cm OctoMap in `base_link`.  The 320 x 240 camera runs at 10 Hz and the
updater subsamples the cloud to keep CPU use reasonable on an integrated-GPU
development machine.

Useful checks while the launch is running:

```bash
ros2 topic list | grep wrist_camera
ros2 topic hz /wrist_camera/depth/points
ros2 run tf2_ros tf2_echo base_link wrist_depth_camera_optical_frame
```

## Tabletop simulation scene

The default Gazebo world contains the work table and Piper arm, but deliberately
contains no panel or physical button. The task consumes only the supplied
world-frame target point and press direction. This keeps planning/control tests
independent of the perception layer.

Run the scene with headless Gazebo and RViz:

```bash
ros2 launch piper_elevator_manipulation elevator_scene.launch.py
```

Show Gazebo's own GUI as well (more CPU/GPU intensive):

```bash
ros2 launch piper_elevator_manipulation elevator_scene.launch.py gazebo_gui:=true
```

Perception is not part of the current planning/control baseline. The earlier
neural-network study (`piper_button_perception`) was removed on 2026-09-28. The
next perception effort targets a **calibration board** and publishes its result
through the same `/press/target_pose` contract, so this package is unaffected by
the change.
