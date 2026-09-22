# Five-axis stator-mounted press tool model

## Physical topology

The pressing assembly is bolted to the joint-6 motor stator. It does not move
with the joint-6 rotor/output flange. The `use_press_tool:=true` Xacro variant
therefore models two fixed branches from `link5` at the original joint-6 frame:

- `link6`, retained at the zero pose as unused rotor/flange geometry;
- `press_tool_link`, carrying the press body, probe, camera, and `press_tcp`.

The active kinematic chain from `base_link` to `press_tcp` contains only
`joint1` through `joint5`.

## Current provisional dimensions

The model is structurally correct but its tool geometry is provisional:

- stator frame in `link5`: copied from the original joint-6 origin;
- tool length to `press_tcp`: 0.1358 m;
- body length: 0.105 m;
- body radius: 0.045 m;
- probe radius: 0.006 m;
- tool mass/inertia: approximate values for simulation stability.

Before real-arm deployment, replace these defaults with CAD or measured values
for the stator mounting transform, body envelope, probe tip, mass, center of
mass, inertia, and camera extrinsics.

## Validation commands

Expand and check the five-axis robot description:

```bash
xacro $(ros2 pkg prefix piper_description)/share/piper_description/urdf/piper_description.xacro \
  use_press_tool:=true -o /tmp/piper_press.urdf
check_urdf /tmp/piper_press.urdf
```

Start the five-axis Gazebo model with physics only:

```bash
ros2 launch piper_gazebo piper_press_gazebo.launch.py gui:=false
```
