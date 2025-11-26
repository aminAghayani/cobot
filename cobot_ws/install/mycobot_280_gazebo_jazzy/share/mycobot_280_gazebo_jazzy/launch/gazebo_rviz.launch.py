import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_ros_gz = FindPackageShare("ros_gz_sim").find("ros_gz_sim")
    pkg_this = FindPackageShare("mycobot_280_gazebo_jazzy").find("mycobot_280_gazebo_jazzy")
    
    # Load robot description with path substitution
    urdf_file = os.path.join(pkg_this, "urdf", "mycobot_280_gazebo.urdf")
    with open(urdf_file, "r") as f:
        urdf_xml = f.read()
    
    # Replace $(find ...) substitution with absolute path for proper resolution
    config_file = os.path.join(pkg_this, "config", "mycobot_280_controllers.yaml")
    urdf_xml = urdf_xml.replace(
        "$(find mycobot_280_gazebo_jazzy)/config/mycobot_280_controllers.yaml",
        config_file
    )
    
    # Create temporary URDF file with resolved paths
    import tempfile
    temp_urdf_fd, temp_urdf_file = tempfile.mkstemp(suffix=".urdf", text=True)
    with open(temp_urdf_file, "w") as f:
        f.write(urdf_xml)
    os.close(temp_urdf_fd)

    # Launch Gazebo with empty world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(pkg_ros_gz, "launch", "gz_sim.launch.py")]
        ),
        launch_arguments={
            "gz_args": "-r empty.sdf",
            "use_sim_time": "true"
        }.items(),
    )

    # Bridge for Gazebo-ROS2 clock synchronization (start early)
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        parameters=[{"use_sim_time": True, "qos_overrides./clock.publisher.durability": "transient_local"}],
        output="screen",
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"use_sim_time": True, "robot_description": urdf_xml}],
        output="screen",
    )

    # Spawn robot in Gazebo using the processed URDF file
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-file", temp_urdf_file, "-name", "mycobot_280"],
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    # RViz2 for visualization and control
    rviz_config_file = os.path.join(
        get_package_share_directory("mycobot_280pi"),
        "config", "mycobot_pi.rviz"
    )
    
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_file],
        parameters=[{"use_sim_time": True}],
        output="screen"
    )

    # Joint State Publisher GUI for interactive control
    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        parameters=[{"use_sim_time": True}],
        output="screen"
    )

    # Spawn joint state broadcaster
    spawn_joint_state_broadcaster = TimerAction(
        period=2.0,
        actions=[Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
            parameters=[{"use_sim_time": True}],
            output="screen",
        )]
    )

    # Spawn joint trajectory controller
    spawn_joint_trajectory_controller = TimerAction(
        period=3.0,
        actions=[Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_trajectory_controller", "--controller-manager", "/controller_manager"],
            parameters=[{"use_sim_time": True}],
            output="screen",
        )]
    )

    return LaunchDescription([
        gazebo,
        clock_bridge,
        robot_state_publisher,
        spawn_robot,
        rviz_node,
        joint_state_publisher_gui,
        spawn_joint_state_broadcaster,
        spawn_joint_trajectory_controller,
    ])