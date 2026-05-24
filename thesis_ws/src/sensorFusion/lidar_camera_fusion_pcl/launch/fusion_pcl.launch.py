"""
launch/fusion_pcl.launch.py

Usage:
  ros2 launch lidar_camera_fusion_pcl fusion_pcl.launch.py bag:=bags/fusion_sensor/Session2Real/Session2Real_0.db3
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    pkg_share = get_package_share_directory("lidar_camera_fusion_pcl")
    params_yaml = os.path.join(pkg_share, "config", "fusion_pcl_params.yaml")

    # ── Launch arguments ────────────────────────────────────────────────────
    bag_arg = DeclareLaunchArgument(
        "bag", default_value="src/bags/fusion_sensor/Session2Real_fixed/Session2Real_fixed_0.db3", description="Path to ROS2 bag file (.db3)"
    )
    target_frame_arg = DeclareLaunchArgument(
        "target_frame",
        default_value="zed_camera_center"#"base_link",
        description="Common TF frame for merged cloud",
    )
    voxel_size_arg = DeclareLaunchArgument(
        "voxel_size", default_value="0.05", description="Voxel leaf size [m] (0 = off)"
    )
    slop_arg = DeclareLaunchArgument(
        "slop", default_value="0.05", description="ApproximateTime slop [s]"
    )
    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz", default_value="true", description="Launch RViz2"
    )
    use_filter_arg = DeclareLaunchArgument(
        "use_filter",
        default_value="true",
        description="Pre-filter ZED cloud before merge",
    )

    # ── Bag playback (delayed so TF is available first) ─────────────────────
    bag_play = TimerAction(
        period=2.0,  # seconds — wait for nodes to spin up
        actions=[
            ExecuteProcess(
                cmd=[
                    "ros2", "bag", "play",
                    LaunchConfiguration("bag"),
                    "--clock",
                    "--loop",
                ],
                output="screen",
                condition=IfCondition(
                    # Only play if bag path was provided
                    PythonExpression(["'", LaunchConfiguration('bag'), "' != ''"])
                ),
            )
        ],
    )

    # ── Pre-filter node for ZED cloud ────────────────────────────────────────
    cloud_filter_node = Node(
        package="lidar_camera_fusion_pcl",
        executable="cloud_filter_node",
        name="cloud_filter_node",
        output="screen",
        parameters=[
            params_yaml
        ],
        condition=IfCondition(LaunchConfiguration("use_filter")),
    )

    # ── Composable container for the fusion node ─────────────────────────────
    fusion_container = ComposableNodeContainer(
        name="fusion_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="lidar_camera_fusion_pcl",
                plugin="lidar_camera_fusion_pcl::CloudMergeNode",
                name="cloud_merge_node",
                parameters=[
                    params_yaml
                ],
                remappings=[],
            )
        ],
        output="screen",
    )

    # ── RViz2 ────────────────────────────────────────────────────────────────
    rviz_cfg = os.path.join(pkg_share, "rviz", "fusion_pcl.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_cfg] if os.path.isfile(rviz_cfg) else [],
        parameters=[{"use_sim_time": True}],
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    return LaunchDescription(
        [
            bag_arg,
            target_frame_arg,
            voxel_size_arg,
            slop_arg,
            use_rviz_arg,
            use_filter_arg,
            cloud_filter_node,
            fusion_container,
            rviz_node,
            bag_play,
        ]
    )