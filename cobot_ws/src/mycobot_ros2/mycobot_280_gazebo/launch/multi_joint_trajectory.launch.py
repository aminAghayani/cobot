from launch import LaunchDescription
from launch.actions import TimerAction, Shutdown
from launch_ros.actions import Node

def generate_launch_description():
    TOTAL_TIME_SEC = 7.5  # (trajectory duration + small margin)

    node = Node(
        package='mycobot_280_gazebo',
        executable='multi_joint_trajectory.py',
        output='screen',
        parameters=[{'controller_name': 'joint_trajectory_controller'}],
        on_exit=Shutdown(),  # <<< when the node exits, close the launch
    )

    # Belt-and-suspenders: hard shutdown shortly after expected finish
    return LaunchDescription([
        node,
        TimerAction(period=TOTAL_TIME_SEC, actions=[Shutdown()]),
    ])

