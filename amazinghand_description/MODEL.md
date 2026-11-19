# AmazingHand models

The left and right assemblies are based on the [Pollen Robotics AmazingHand CAD
and simulation files](https://github.com/pollen-robotics/AmazingHand/tree/3e8241074df3436a3044ced4881e3bb2133aa725).
The mechanical design and meshes are by Pollen Robotics and are licensed under
[CC BY 4.0](https://creativecommons.org/licenses/by/4.0/). The upstream MJCF is
licensed under [Apache-2.0](https://www.apache.org/licenses/LICENSE-2.0).
The adapted models use the CAD joint poses, add palm and top covers and external
contact surfaces, and estimate the wrist inertia including the eight servos.

## ROS 2 and Gazebo Harmonic

```sh
ros2 launch amazinghand_bringup hand.launch.py use_sim:=true
ros2 launch amazinghand_bringup hand.launch.py use_sim:=true hand_name:=left_hand gui:=false
```

The URDF represents the distal and coupler hinges as passive mimic joints
linearized around the CAD reference pose. The standalone models below include
closed-loop linkage geometry. Use `show_covers:=false` when expanding the Xacro
to inspect the internal geometry.

## MuJoCo

```sh
share="$(ros2 pkg prefix --share amazinghand_description)"
python -m mujoco.viewer --mjcf="$share/mjcf/right_hand.xml"
```

Use `left_hand.xml` for the left hand. Motor names range from
`finger1_motor1` to `finger4_motor2`, in index, middle, ring, and thumb order.

## Gazebo Classic with ODE

Run these commands from a shell with the Gazebo Classic CLI. The supplied ODE
world configures the solver and time step for the closed linkage.

```sh
share="$(ros2 pkg prefix --share amazinghand_description)"
export GAZEBO_MODEL_PATH="$(dirname "$share")${GAZEBO_MODEL_PATH:+:$GAZEBO_MODEL_PATH}"
gazebo -e ode "$share/sdf/closed_loop.world"
```

In another shell with the same environment, spawn either hand and give its
motors position targets to hold against gravity.

```sh
share="$(ros2 pkg prefix --share amazinghand_description)"
gz model --spawn-file "$share/sdf/right_hand.sdf" --model-name right_hand -z 0.2
for finger in 1 2 3 4; do
  for motor in 1 2; do
    gz joint -m right_hand -j "finger${finger}_motor${motor}" --pos-t 0 --pos-p 0.5 --pos-d 0.01
  done
done
```
