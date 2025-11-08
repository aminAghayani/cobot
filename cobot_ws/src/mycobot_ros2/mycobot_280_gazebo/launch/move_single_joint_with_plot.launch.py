from launch import LaunchDescription
from launch.actions import TimerAction, Shutdown
from launch_ros.actions import Node
import os
from datetime import datetime

def generate_launch_description():
    # >>> EDIT THESE LINES <<<
    CONTROLLER = 'joint_trajectory_controller'
    JOINT      = 'joint2_to_joint1'
    POSITION   = 0.5235987755982988  # 30° rad
    DURATION   = 4.0                 # seconds
    BUFFER     = 2.0                 # record extra seconds
    # Save directory (change if you like)
    SAVE_DIR   = os.path.expanduser('~/mycobot_ws/joint_movement_plots')
    # Timestamped filenames per run
    STAMP      = datetime.now().strftime('%Y%m%d-%H%M%S')
    # >>> STOP EDITING HERE <<<

    total_time = DURATION + BUFFER

    recorder = Node(
        package='mycobot_280_gazebo',
        executable='plot_joint_feedback.py',
        output='screen',
        parameters=[{
            'joint_name': JOINT,
            'record_secs': total_time,
            'out_csv': f'{SAVE_DIR}/{STAMP}_{JOINT}_feedback.csv',
            'out_png': f'{SAVE_DIR}/{STAMP}_{JOINT}_feedback.png',
            'show_plot': False,
        }],
        on_exit=Shutdown(),
    )

    mover = Node(
        package='mycobot_280_gazebo',
        executable='move_single_joint.py',
        output='screen',
        parameters=[{
            'controller_name': CONTROLLER,
            'joint_name': JOINT,
            'position': POSITION,
            'duration': DURATION,
            'goal_tolerance': 0.01,
            'settle_velocity': 0.02,
            'max_wait': 1.0,
        }],
    )

    return LaunchDescription([
        recorder,
        TimerAction(period=0.2, actions=[mover]),
        TimerAction(period=total_time + 3.0, actions=[Shutdown()]),
    ])

