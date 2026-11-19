# amazinghand_ros2

ROS2 controller scaffold for the [Pollen Robotics AmazingHand](https://github.com/pollen-robotics/AmazingHand).

## Package layout

```
amazinghand_ros2/
├── amazinghand_msgs/          # FingerCommand, HandCommand message types
├── amazinghand_hardware/      # ros2_control SystemInterface + SCS0009 driver
├── amazinghand_controller/    # ChainableController (semantic → HW interfaces)
├── amazinghand_description/   # URDF/xacro, parametric finger macro
└── amazinghand_bringup/       # Launch files + runtime YAML config
```

---

## Core design: rules-table driven joint "steering" 

Many humanoid hands present a motor-and-cable or motor-per-joint actuation
mechanism. The AH uses a dual motor-to-linkage method. This requires a different
actuation system vs. the typical humanoid hand software control. 

Fortunately, ROS provides a number of hardware controllers that drive similar
systems. While they are not direct drop-ins, enough of the math is the same that
we can borrow from them, saving us from having to invent YetAnotherHardwareController. 

Each finger has two SCS0009 servos (leader + follower) driving a parallel
mechanism that produces two independent output axes. The "leader" servo receives
the actuation message and the "follower" either copies or inverts the value sent to the leader.

| Desired motion      | Leader | Follower | Rule         |
|---------------------|--------|----------|--------------|
| Flexion/Extension   | +cmd   | −cmd     | Opposition   |
| Abduction/Adduction | +cmd   | +cmd     | Copy         |

Combined into a 2×2 mixing matrix applied in `finger_types.hpp`:

```
motor_leader   =  flexion_cmd + (chirality × abduction_cmd)
motor_follower = −flexion_cmd + (chirality × abduction_cmd)
```

Inverse (for state feedback from servo positions):

```
flexion_rad    = (leader_rad − follower_rad) / 2
abduction_rad  = (leader_rad + follower_rad) / 2 × chirality
```

`chirality` = +1.0 for right hand, −1.0 for left hand.

The mixing lives entirely in the hardware interface. Everything above it
(controller, MoveIt, teleop) works in clean semantic `[flexion, abduction]`
coordinates and never touches raw servo IDs.

---

## Adding or removing fingers

Keep the model and controller configuration in sync when changing the finger set:

1. In `amazinghand_description/config/<hand_name>.yaml`, update the finger geometry
   and joint poses under `fingers` for the affected hand.
2. In `amazinghand_description/urdf/hand.urdf.xacro`, update the matching finger
   macro call and the flexion, abduction, and passive joint declarations in
   `ros2_control`. This Xacro is shared by both hands.
   For real hardware, update the `finger.N.*` parameters and servo IDs. Number
   those entries consecutively starting at `0`.
3. In `amazinghand_bringup/config/<hand_name>.yaml`, update `finger_names` for
   the affected hand. If you use Gazebo with `rqt:=true`, also update the
   trajectory controller's `joints` list in
   `amazinghand_bringup/config/gazebo_controllers.yaml`.

---

## Joint naming convention

All joint names follow the pattern:

```
<hand_name>_<finger_name>_<axis>
```

Examples:
- `right_hand_index_flexion`
- `right_hand_thumb_abduction`
- `left_hand_middle_flexion`

---

## Build

```bash
cd ~/ros2_ws/src
ln -s /path/to/amazinghand_ros2 .
cd ~/ros2_ws
colcon build --packages-select \
    amazinghand_msgs \
    amazinghand_hardware \
    amazinghand_controller \
    amazinghand_description \
    amazinghand_bringup
source install/setup.bash
```

---

## Launch

**Real hardware (right hand):**
```bash
ros2 launch amazinghand_bringup hand.launch.py
```

**Real hardware (left hand):**
```bash
ros2 launch amazinghand_bringup hand.launch.py \
    hand_name:=left_hand serial_port:=/dev/ttyUSB1
```

**Gazebo simulation:**
```bash
ros2 launch amazinghand_bringup hand.launch.py use_sim:=true
```

Add `hand_name:=left_hand` for the left hand or `gui:=false` to run headless.

**Gazebo simulation with trajectory sliders:**
```bash
ros2 launch amazinghand_bringup hand.launch.py use_sim:=true rqt:=true
```

In rqt, select `/controller_manager` and `hand_trajectory_controller`.

See the [model documentation](amazinghand_description/MODEL.md) for CAD
attribution and the standalone MuJoCo and Gazebo Classic models.

---

## Sending commands

Publish a `HandCommand` to command one or more fingers:

```bash
ros2 topic pub /amazinghand_controller/command \
    amazinghand_msgs/msg/HandCommand \
    '{fingers: [{finger_name: "index", flexion_rad: 1.0, abduction_rad: 0.0}]}'
```

Fingers not mentioned in the message retain their current commanded position.

---

## What still needs implementing

| File | Status | Notes |
|------|--------|-------|
| `amazinghand_description/rviz/hand.rviz` | TODO | RViz config for visualisation |
| Teleop bridge | TODO | Adapt from rover teleop interface |
| MoveIt config | Future | Standard `moveit_setup_assistant` workflow once URDF is solid |
