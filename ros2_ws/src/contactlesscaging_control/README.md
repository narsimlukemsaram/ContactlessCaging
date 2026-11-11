# ROS 2 integration support

This package provides a Python ROS 2 Humble control workflow corresponding to the control architecture presented in the submitted manuscript: independent PID loops, leader-referenced global offsets, backward leader movement, forward follower movement, `/cmd_vel` generation, a 100 cm straight reference, and a parser for the historic PhaseSpace UDP packet format visible in the C++ code.

All four robot controllers start **disabled**. Enable each only after checking world-frame poses, global offsets, axes, robot namespaces, `cmd_vel` mapping, heading units and emergency-stop behaviour. The sample offsets illustrate configuration syntax and are **not measured robot placements**.

## Build

```bash
source /opt/ros/humble/setup.bash
sudo apt update
sudo apt install python3-colcon-common-extensions python3-yaml ros-humble-geometry-msgs ros-humble-std-srvs
cd ros2_ws
colcon build --symlink-install --packages-select contactlesscaging_control
source install/setup.bash
```

## Run

When `/phasespace/<robot>/pose` is already published, use:

```bash
ros2 launch contactlesscaging_control four_robots.launch.py
```

For the old C++ UDP tracker-packet format, use the bridge:

```bash
ros2 launch contactlesscaging_control four_robots.launch.py use_udp_bridge:=true
```

Check poses and controllers before applying motion. After configuring measured offsets, safe motion limits and the real robot `cmd_vel` topics, enable controllers:

```bash
ros2 param set /robot1/formation_controller enabled true
ros2 param set /robot2/formation_controller enabled true
ros2 param set /robot3/formation_controller enabled true
ros2 param set /robot4/formation_controller enabled true
ros2 service call /formation/start std_srvs/srv/Trigger '{}'
```

To stop each robot's controller, set `enabled false`. The local controller publishes a zero `Twist` whenever disabled or when its pose stream is stale.

## Interfaces

| ROS 2 interface | Message type | Function |
|---|---|---|
| `/phasespace/robot1/pose` to `robot4/pose` | `geometry_msgs/PoseStamped` | Calibrated global poses, metres |
| `/formation/leader_target` | `geometry_msgs/PoseStamped` | Straight-line target |
| `/<robot>/cmd_vel` | `geometry_msgs/Twist` | Local differential-drive commands |
| `/formation/start` | `std_srvs/Trigger` | Start reference from live leader position |

The package is designed to be adapted to the original experimental data paths and controllers. Acoustic phase streaming and FPGA–IR firmware remain separate from these ROS 2 nodes.
