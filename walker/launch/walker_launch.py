from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction, RegisterEventHandler, DeclareLaunchArgument
from launch.event_handlers.on_process_start import OnProcessStart
from launch.substitutions import FindExecutable, LaunchConfiguration
from launch.conditions import IfCondition
import datetime

def generate_launch_description():
    # Declare launch argument for enabling/disabling rosbag recording
    record_arg = DeclareLaunchArgument(
        'record',
        default_value='false',
        description='Whether to record rosbag or not'
    )
    
    # Get current timestamp for the bag file name
    timestamp = datetime.datetime.now().strftime('%Y_%m_%d-%H_%M_%S')
    
    # Walker node
    walker_node = Node(
        package='walker',
        executable='robot_control',
        name='walker',
        output='screen',
        parameters=[{
            'use_sim_time': True  # Important for Isaac Sim time synchronization
        }]
    )
    
    # ROS bag recording process - updated topics for Isaac Sim
    rosbag_record = ExecuteProcess(
        condition=IfCondition(LaunchConfiguration('record')),
        cmd=[[
            FindExecutable(name='ros2'),
            ' bag record',
            ' /cmd_vel',
            ' /back_2d_lidar/scan',
            ' /chassis/odom',
            ' /chassis/imu',
            ' -o carter_walker_recording_',
            timestamp,
        ]],
        shell=True
    )
    
    # Timer to stop recording after 15 seconds
    timer_action = TimerAction(
        period=15.0,
        actions=[
            ExecuteProcess(
                cmd=[[
                    FindExecutable(name='ros2'),
                    ' bag shutdown'
                ]],
                shell=True
            )
        ],
        condition=IfCondition(LaunchConfiguration('record'))
    )
    
    # Register event handler to start the timer when rosbag recording starts
    start_timer_event = RegisterEventHandler(
        condition=IfCondition(LaunchConfiguration('record')),
        event_handler=OnProcessStart(
            target_action=rosbag_record,
            on_start=[timer_action]
        )
    )
    
    return LaunchDescription([
        record_arg,
        walker_node,
        rosbag_record,
        start_timer_event
    ])
