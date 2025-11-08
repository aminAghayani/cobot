from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    yaml = """joint_names:
- joint2_to_joint1
- joint3_to_joint2
- joint4_to_joint3
- joint5_to_joint4
- joint6_to_joint5
- joint6output_to_joint6
points:
- positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  time_from_start:
    sec: 4
    nanosec: 0
"""
    return LaunchDescription([
        ExecuteProcess(
            cmd=[
                'ros2', 'topic', 'pub', '--once',
                '/joint_trajectory_controller/joint_trajectory',
                'trajectory_msgs/msg/JointTrajectory',
                yaml
            ],
            output='screen'
        )
    ])

