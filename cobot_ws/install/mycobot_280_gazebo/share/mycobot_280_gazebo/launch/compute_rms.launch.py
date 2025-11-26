from launch import LaunchDescription
from launch.actions import TimerAction, Shutdown
from launch_ros.actions import Node
import os
from datetime import datetime

def generate_launch_description():
    CONTROLLER = 'joint_trajectory_controller'
    SECS       = 20.0  # safe upper bound
    SAVE_DIR   = os.path.expanduser('~/mycobot_ws/joint_movement_plots')
    STAMP      = datetime.now().strftime('%Y%m%d-%H%M%S')

    logger = Node(
        package='mycobot_280_gazebo',
        executable='log_rms_tracking.py',  # <-- your fallback code lives here
        output='screen',
        parameters=[{
            'controller_name': CONTROLLER,
            'record_secs': SECS,
            'out_csv': f'{SAVE_DIR}/{STAMP}_rms_tracking.csv',
            # 'joints': ['joint2_to_joint1','joint3_to_joint2'],
        }],
        on_exit=Shutdown(),  # normal path: node ends -> launch ends
    )

    # Hard-stop a touch after the record window, just in case
    return LaunchDescription([
        logger,
        TimerAction(period=SECS + 1.0, actions=[Shutdown()]),
    ])

