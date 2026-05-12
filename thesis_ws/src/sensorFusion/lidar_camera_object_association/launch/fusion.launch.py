"""
fusion.launch.py  –  lidar_camera_object_association (C++ build)
----------------------------------------------------------------
Launches:
  1. association_node  (C++ executable, params from params.yaml)
  2. ros2 bag play     (with --clock so sim-time is consistent)
  3. rviz2             (delayed 2 s, pre-configured fusion.rviz)

Override any argument from the CLI, e.g.:
  ros2 launch lidar_camera_object_association fusion.launch.py \
      bag_path:=/data/my_recording_0.db3 \
      sync_slop:=0.15
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    TimerAction,
    LogInfo,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

_PKG = "lidar_camera_object_association"

# Default bag path – relative to workspace root
_WS_ROOT = Path(__file__).resolve().parents[4]   # ws
_DEFAULT_BAG = str(
    _WS_ROOT / "src" / "bags" / "fusion_sensor" / "session2Real_fixed"
)
print("WS ROOT:", _WS_ROOT)
print("BAG PATH:", _DEFAULT_BAG)


def generate_launch_description() -> LaunchDescription:

    # ── Declare overridable arguments ─────────────────────────────────────
    bag_path_arg = DeclareLaunchArgument(
        "bag_path",
        default_value=_DEFAULT_BAG,
        description="Absolute path to the .db3 ROS 2 bag file.",
    )
    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=PathJoinSubstitution(
            [FindPackageShare(_PKG), "config", "params.yaml"]
        ),
        description="Path to the params.yaml configuration file.",
    )
    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value=PathJoinSubstitution(
            [FindPackageShare(_PKG), "rviz", "fusion.rviz"]
        ),
        description="Path to the RViz2 .rviz profile.",
    )
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use bag /clock instead of wall clock.",
    )

    # ── 1. Association node (C++) ──────────────────────────────────────────
    association_node = Node(
        package=_PKG,
        executable="association_node",
        name="association_node",
        output="screen",
        parameters=[
            LaunchConfiguration("params_file"),
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
        remappings=[],
    )

    # ── 2. Bag playback ───────────────────────────────────────────────────
    bag_play = ExecuteProcess(
        cmd=[
            "ros2", "bag", "play",
            LaunchConfiguration("bag_path"),
            "--clock",
            "--loop",
            "--storage", "sqlite3",
        ],
        output="screen",
        name="bag_play",
    )

    # ── 3. RViz2 (delayed 2 s) ────────────────────────────────────────────
    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", LaunchConfiguration("rviz_config")],
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        output="screen",
    )
    rviz2_delayed = TimerAction(period=2.0, actions=[rviz2])

    # ── Log ───────────────────────────────────────────────────────────────
    log = LogInfo(msg=["[fusion] Playing bag: ", LaunchConfiguration("bag_path")])

    return LaunchDescription([
        bag_path_arg,
        params_file_arg,
        rviz_config_arg,
        use_sim_time_arg,
        log,
        association_node,
        bag_play,
        rviz2_delayed,
    ])
