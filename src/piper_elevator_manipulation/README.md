# Piper elevator manipulation demo

`press_button_demo` is a deliberately small MoveIt Task Constructor (MTC)
baseline for the planning/control layer.  It plans the following sequence from
the live Gazebo joint state:

1. plan to a parameterized pre-press pose;
2. move `press_distance` metres along `+X` in `base_link`;
3. retreat by the same distance along `-X`.

The target is supplied explicitly in the planning frame through launch
parameters.  This package deliberately has no dependency on the paused
perception prototype.  A later task interface will accept a world-frame point
and press direction without coupling the planning layer to any particular
target producer.

Run a dry plan first:

```bash
ros2 launch piper_elevator_manipulation press_button_demo.launch.py
```

Then execute the trajectory in Gazebo:

```bash
ros2 launch piper_elevator_manipulation press_button_demo.launch.py execute:=true
```

The launch starts Gazebo without its own GUI and starts MoveIt RViz.  It also
loads MTC's `ExecuteTaskSolutionCapability`, which forwards the planned
trajectory to the existing `arm_controller`.

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

## Elevator table scene

The default Gazebo world contains a work table, the Piper arm, and a vertical
elevator panel with three physical button discs.  From top to bottom they are
blue, red, and green.  Each is a separately named Gazebo model so its pose can
serve as synthetic-data ground truth.  The front face of the red middle button
is aligned with the demo target at `(0.300, 0.000, 0.240)` in `base_link`.

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
