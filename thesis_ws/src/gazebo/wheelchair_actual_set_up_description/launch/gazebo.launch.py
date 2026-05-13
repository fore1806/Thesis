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

    declare_rviz = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Set to false to skip launching RViz')

    declare_world = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(pkg, 'worlds', 'wheelchair_world.sdf'),
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
            '-z', '0.002',
            '-R', '0.0',
            '-P', '0.0',
            '-Y', '0.0',
        ],
    )

    # 4. ROS ↔ Gazebo topic bridge
    #    /cmd_vel: ROS→Gz  (drives the native DiffDrive plugin)
    #    /odom:   Gz→ROS   (published by the native DiffDrive plugin)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        parameters=[{
            'config_file': os.path.join(pkg, 'config', 'ros_gz_bridge.yaml'),
            'use_sim_time': True,
        }],
    )

    # 5. Odom → base_footprint TF broadcaster
    odom_tf_broadcaster = Node(
        package='wheelchair_actual_set_up_description',
        executable='odom_tf_broadcaster',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # 6. Static world geometry markers for RViz
    world_markers = Node(
        package='wheelchair_actual_set_up_description',
        executable='world_markers_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # 7. RViz2 visualization (optional)
    rviz_config = os.path.join(pkg, 'config', 'wheelchair_sim.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([
        declare_gui,
        declare_rviz,
        declare_world,
        gazebo,
        robot_state_publisher,
        spawn_robot,
        bridge,
        odom_tf_broadcaster,
        world_markers,
        rviz,
    ])
