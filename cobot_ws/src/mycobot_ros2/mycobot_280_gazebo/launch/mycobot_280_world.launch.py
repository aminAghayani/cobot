from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    pkg = FindPackageShare("mycobot_280_gazebo").find("mycobot_280_gazebo")
    urdf = PathJoinSubstitution([pkg, "urdf", "mycobot_280_gazebo.urdf"])
    controllers = PathJoinSubstitution([pkg, "config", "mycobot_280_controllers.yaml"])

    return LaunchDescription([
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[{"robot_description": open(str(urdf)).read()}],
        ),
        ExecuteProcess(
            cmd=["ros2", "launch", "mycobot_280_gazebo", "mycobot_280_gazebo.launch.py"],
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_trajectory_controller", "--controller-manager", "/controller_manager"],
            output="screen",
        ),
    ])

