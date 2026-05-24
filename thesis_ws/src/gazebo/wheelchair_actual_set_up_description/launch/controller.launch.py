"""
controller.launch.py  —  ROS 2 Humble replacement for controller.launch (ROS 1)

What changed vs the ROS 1 version:
  - rosparam file load  →  parameters= in Node()
  - controller_manager spawner syntax updated for ros2_control
  - EffortJointInterface / JointPositionController removed entirely
  - Only drive wheels get controllers; casters are passive (no entry)
  - joint_state_controller  →  joint_state_broadcaster
  - Effort position controllers  →  DiffDriveController (velocity)
  - Spaced joint names ("Revolute 1")  →  underscore names required

Use this launch file if you want to start the controllers independently
from Gazebo (e.g. attaching to an already-running simulation).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node


def generate_launch_description():

    pkg = get_package_share_directory('wheelchair_actual_set_up_description')
    controllers_yaml = os.path.join(pkg, 'config', 'ros2_controllers.yaml')

    # ── Robot state publisher ─────────────────────────────────────────────────
    # Needed so TF frames are published from /joint_states
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}],
        # No remapping needed in ROS 2 — topic is /joint_states by default
    )

    # ── Controller manager ────────────────────────────────────────────────────
    # In Gazebo simulation the controller_manager is started by the gz_ros2_control
    # plugin automatically. This node is only needed for real-hardware bringup.
    # Comment it out when running with Gazebo.
    # controller_manager = Node(
    #     package='controller_manager',
    #     executable='ros2_control_node',
    #     parameters=[controllers_yaml],
    #     output='screen',
    # )

    # ── Spawner: joint_state_broadcaster ─────────────────────────────────────
    # Must be active before any motion controller starts
    load_jsb = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
        ],
        output='screen',
    )

    # ── Spawner: diff_drive_controller (drive wheels only) ────────────────────
    # Controls left_wheel_joint + right_wheel_joint via /cmd_vel
    # Caster joints (Revolute 3–13) are NOT listed → passive/uncontrolled
    load_diff = TimerAction(
        period=1.5,  # wait for joint_state_broadcaster to finish loading
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                'diff_drive_controller',
                '--controller-manager', '/controller_manager',
            ],
            output='screen',
        )],
    )

    return LaunchDescription([
        robot_state_publisher,
        load_jsb,
        load_diff,
    ])
