"""
gazebo.launch.py  —  ROS 2 Humble replacement for gazebo.launch (ROS 1)

Equivalent to the original:
  <include file="$(find gazebo_ros)/launch/empty_world.launch"/>
  <node pkg="gazebo_ros" type="spawn_model" .../>

Now uses ros_gz_sim (Gazebo Harmonic/Fortress bridge for ROS 2 Humble).
"""

import os
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg = get_package_share_directory('wheelchair_actual_set_up_description')

    # ── Arguments ────────────────────────────────────────────────────────────
    declare_gui = DeclareLaunchArgument(
        'gui', default_value='true',
        description='Set to false to run Gazebo headless')

    declare_world = DeclareLaunchArgument(
        'world', default_value='empty.sdf',
        description='Gazebo world file to load')

    # ── Robot description (xacro → string) ───────────────────────────────────
    xacro_file = os.path.join(pkg, 'urdf', 'wheelchair_actual_set_up.xacro')
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    # ── Nodes ─────────────────────────────────────────────────────────────────

    # 1. Gazebo simulator
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('ros_gz_sim'),
            '/launch/gz_sim.launch.py'
        ]),
        launch_arguments={
            'gz_args': ['-r ', LaunchConfiguration('world')],
            'on_exit_shutdown': 'true',
        }.items(),
    )

    # 2. Robot state publisher — publishes /robot_description and TF tree
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_raw,
            'use_sim_time': True,
        }],
    )

    # 3. Spawn the robot into Gazebo (reads /robot_description topic)
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-name',  'wheelchair',
            '-topic', '/robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.05',   # slight lift so it doesn't clip the ground plane
            '-R', '1.5708', # Corect the bad orientation
            '-P', '0.0',
            '-Y', '0.0',
        ],
    )

    # 4. ROS ↔ Gazebo topic bridge
    #    Format: <gz_topic>@<ros_msg_type>[<gz_msg_type>
    #    [ = gz→ros,  ] = ros→gz,  @ = bidirectional
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/scan_front@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            '/scan_rear@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
        ],
        parameters=[{'use_sim_time': True}],
    )

    # 5. Controller spawner — delayed 3 s to let Gazebo finish loading
    #    joint_state_broadcaster must start before diff_drive_controller
    load_joint_state_broadcaster = TimerAction(
        period=3.0,
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
            output='screen',
        )],
    )

    load_diff_drive_controller = TimerAction(
        period=4.0,
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            arguments=['diff_drive_controller', '--controller-manager', '/controller_manager'],
            output='screen',
        )],
    )

    return LaunchDescription([
        declare_gui,
        declare_world,
        gazebo,
        robot_state_publisher,
        spawn_robot,
        bridge,
        load_joint_state_broadcaster,
        load_diff_drive_controller,
    ])
