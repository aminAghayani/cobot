import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_this = get_package_share_directory("mycobot_280_gazebo_jazzy")
    
    # Load robot description
    urdf_file = os.path.join(pkg_this, "urdf", "mycobot_280_gazebo.urdf")
    with open(urdf_file, "r") as f:
        urdf_xml = f.read()
    
    # Replace $(find ...) substitution with absolute path for proper resolution
    config_file = os.path.join(pkg_this, "config", "mycobot_280_controllers.yaml")
    urdf_xml = urdf_xml.replace(
        "$(find mycobot_280_gazebo_jazzy)/config/mycobot_280_controllers.yaml",
        config_file
    )

    # Robot State Publisher - essential for RViz to display the robot
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"use_sim_time": False, "robot_description": urdf_xml}],
        output="screen",
    )
    
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
        parameters=[{"use_sim_time": False}],
        output="screen"
    )

    # Joint State Publisher GUI for interactive control sliders
    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        parameters=[{"use_sim_time": False}],
        output="screen"
    )

    return LaunchDescription([
        robot_state_publisher,
        rviz_node,
        joint_state_publisher_gui,
    ])