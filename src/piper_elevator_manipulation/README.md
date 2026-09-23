# Piper elevator manipulation demo

`press_button_demo` is the first five-axis MoveIt Task Constructor (MTC)
baseline for the planning/control layer. It plans the following sequence from
the live Gazebo joint state:

1. capture `joint1` through `joint5` as the return state;
2. solve a five-dimensional pre-press target and plan a collision-free approach;
3. advance to contact and move through the configured press depth;
4. retreat to the pre-press waypoint;
5. restore normal collision checking and return to the captured start state.

The target is supplied explicitly in `world` through launch parameters. Its
task contract is a point plus a press direction, not an arbitrary 6D pose:
rotation around the cylindrical press rod is intentionally free because J6 is
not actuated. `FiveDofIkSolver` numerically minimizes TCP position error and
press-axis direction error, then MTC performs collision checking and trajectory
planning. The package has no dependency on the paused perception prototype.

Run a dry plan first:

```bash
ros2 launch piper_elevator_manipulation press_button_demo.launch.py
```

Then execute the trajectory in Gazebo:

```bash
ros2 launch piper_elevator_manipulation press_button_demo.launch.py execute:=true
```

The default target is the virtual world-frame point `(0.300, 0.000, 0.240)`
with press direction `(+1, 0, 0)`, a 30 mm pre-press clearance, and 4 mm
press depth. No physical panel or button is present in the default simulation
world. Override `button_x`, `button_y`, `button_z`, `press_direction_x`,
`press_direction_y`, `press_direction_z`, `approach_distance`, or
`press_depth` as needed. The launch starts the five-axis Gazebo model, the
`piper_press_moveit` configuration, RViz, and MTC's execution capability.

The custom IK tolerances, numerical restart count, speed limits, and provisional
tool dimensions remain development parameters. They must be calibrated and
validated before commanding hardware.

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
independent of the paused perception prototype.

Run the scene with headless Gazebo and RViz:

```bash
ros2 launch piper_elevator_manipulation elevator_scene.launch.py
```

Show Gazebo's own GUI as well (more CPU/GPU intensive):

```bash
ros2 launch piper_elevator_manipulation elevator_scene.launch.py gazebo_gui:=true
```

The earlier perception study is retained for reference in
[`docs/button_perception_plan.md`](docs/button_perception_plan.md), but it is
paused and is not part of the current planning/control baseline.
