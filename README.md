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

## Core design: the mixing matrix

Each finger has two SCS0009 servos (leader + follower) driving a parallel
mechanism that produces two independent output axes.

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

Edit **two** files in lock-step:

1. `amazinghand_bringup/config/right_hand.yaml` — add/remove a finger entry
2. `amazinghand_description/urdf/hand.urdf.xacro` — add/remove the matching
   `<xacro:amazinghand_finger .../>` block and its `<joint>` + `<param>` entries

There is no code to change. A 3-finger hand, a 5-finger hand, and the default
4-finger hand are all identical in structure.

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
| `scs0009_driver.cpp` | **TODO** | Serial I/O stubs — see inline comments for protocol |
| `amazinghand_description/rviz/hand.rviz` | TODO | RViz config for visualisation |
| `amazinghand_description/meshes/` | TODO | STL/DAE from upstream CAD |
| Teleop bridge | TODO | Adapt from rover teleop interface |
| MoveIt config | Future | Standard `moveit_setup_assistant` workflow once URDF is solid |
| Gazebo sim plugin | Future | `gz_ros2_control` wiring once description is validated |

The serial driver is the only piece that touches hardware-specific code.
Everything else compiles and runs against the stub immediately.
