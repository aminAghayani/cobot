from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import os

def generate_launch_description():
    pkg_ros_gz = FindPackageShare("ros_gz_sim").find("ros_gz_sim")
    pkg_this   = FindPackageShare("mycobot_280_gazebo").find("mycobot_280_gazebo")

    # Plain string paths
    urdf_file = os.path.join(pkg_this, "urdf", "mycobot_280_gazebo.urdf")
    with open(urdf_file, "r") as f:
        urdf_xml = f.read()

    # Gazebo Sim (Ignition) – empty world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(pkg_ros_gz, "launch", "gz_sim.launch.py")]
        ),
        launch_arguments={"gz_args": "-r empty.sdf"}.items(),
    )

    # Robot State Publisher – publishes /robot_description
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"use_sim_time": True, "robot_description": urdf_xml}],
        output="screen",
    )

    # Spawn the URDF into Gazebo (use a substitution here is fine)
    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-file", os.path.join(pkg_this, "urdf", "mycobot_280_gazebo.urdf"),
                   "-name", "mycobot_280"],
        output="screen",
    )

    # Spawn controllers AFTER gz_ros2_control starts inside Gazebo
    spawn_js = TimerAction(
        period=2.0,
        actions=[Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster",
                       "--controller-manager", "/controller_manager"],
            output="screen",
        )]
    )

    spawn_traj = TimerAction(
        period=3.0,
        actions=[Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_trajectory_controller",
                       "--controller-manager", "/controller_manager"],
            output="screen",
        )]
    )

    return LaunchDescription([gazebo, rsp, spawn, spawn_js, spawn_traj])

