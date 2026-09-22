# Piper five-axis press MoveIt configuration

This package is the MoveIt 2 model for the stator-mounted press tool. It is
kept separate from `piper_with_gripper_moveit`, so the upstream six-axis
gripper configuration remains available for comparison and regression tests.

## Robot model contract

- Planning group: `arm`
- Active joints: `joint1` through `joint5`
- Fixed joints: `joint6_fixed`, `press_tool_fixed`, `press_tcp_fixed`
- Planning tip / task TCP: `press_tcp`
- Trajectory controller: `arm_controller` with five joints
- Planning frame: `base_link` (the URDF is fixed to `world`)

The tool dimensions and mounting transform are provisional until the CAD model
or measured dimensions are available. See
`piper_description/docs/press_tool_model.md` for the current assumptions.

## Run with Gazebo

Start the matching five-axis Gazebo model first:

```bash
ros2 launch piper_gazebo piper_press_gazebo.launch.py gui:=false
```

Then start MoveIt and RViz in a second terminal:

```bash
ros2 launch piper_press_moveit piper_press_moveit.launch.py rviz:=true
```

For a headless smoke test, set `rviz:=false`.

## Five-axis planning constraint

The arm can control the TCP position and press-axis direction, but cannot
independently control rotation about that axis. The current KDL plugin is used
as a baseline IK solver. Higher-level press planning must therefore generate or
sample the free roll angle instead of treating an arbitrary six-dimensional
pose as mandatory. This policy belongs in the task-planning layer and should
not be hidden by reintroducing `joint6` as an actuated joint.
