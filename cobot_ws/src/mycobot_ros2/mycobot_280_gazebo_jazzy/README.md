# MyCobot 280 Gazebo Jazzy

Minimal Gazebo simulation package for MyCobot 280 optimized for ROS2 Jazzy.

## Quick Start

1. Build the package:
```bash
cd /home/amin/cobot/cobot_ws
colcon build --packages-select mycobot_280_gazebo_jazzy
source install/setup.bash
```

2. Launch Gazebo simulation:
```bash
ros2 launch mycobot_280_gazebo_jazzy mycobot_280_gazebo_jazzy.launch.py
```

3. Send commands to the robot using joint trajectory controller:
```bash
# List available controllers
ros2 control list_controllers

# Send joint commands (example)
ros2 topic pub /joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "
header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: ''
joint_names:
- joint1
- joint2  
- joint3
- joint4
- joint5
- joint6
points:
- positions: [0.5, 0.5, 0.5, 0.5, 0.5, 0.5]
  time_from_start:
    sec: 2
    nanosec: 0
"
```

## Features

- Minimal setup for ROS2 Jazzy compatibility
- Joint trajectory controller for commanding the robot
- Joint state broadcasting for robot state feedback
- Empty world Gazebo simulation