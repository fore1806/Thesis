"""
display.launch.py  —  ROS 2 Humble replacement for display.launch (ROS 1)

Launches robot_state_publisher + joint_state_publisher_gui + RViz2.
Use this for visualising/checking the URDF without Gazebo.
"""

import os
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg = get_package_share_directory('wheelchair_actual_set_up_description')

    # ── Arguments ────────────────────────────────────────────────────────────
    declare_gui = DeclareLaunchArgument(
        'gui', default_value='true',
        description='Set to true to use joint_state_publisher_gui sliders')

    declare_rviz = DeclareLaunchArgument(
        'rvizconfig',
        default_value=os.path.join(pkg, 'launch', 'urdf.rviz'),
        description='Path to RViz2 config file')

    # ── Robot description ─────────────────────────────────────────────────────
    xacro_file = os.path.join(pkg, 'urdf', 'wheelchair_actual_set_up.xacro')
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    # ── Nodes ─────────────────────────────────────────────────────────────────

    # Publishes /robot_description and computes TF from joint states
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw}],
    )

    # GUI sliders to manually move joints — useful for URDF inspection
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen',
    )

    # RViz2 — note: class names changed from rviz/ to rviz_default_plugins/
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    return LaunchDescription([
        declare_gui,
        declare_rviz,
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz,
    ])
