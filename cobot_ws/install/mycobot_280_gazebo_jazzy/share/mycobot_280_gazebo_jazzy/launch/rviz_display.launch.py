import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # RViz configuration file path
    rviz_config_file = os.path.join(
        get_package_share_directory("mycobot_280pi"),
        "config", "mycobot_pi.rviz"
    )
    
    # RViz2 node
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_file],
        parameters=[{"use_sim_time": True}],
        output="screen"
    )

    # Joint State Publisher GUI for interactive control sliders
    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        parameters=[{"use_sim_time": True}],
        output="screen"
    )

    return LaunchDescription([
        rviz_node,
        joint_state_publisher_gui,
    ])