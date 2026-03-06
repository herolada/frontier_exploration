"""
Launch file for wfd_explorer.

All tunable parameters are exposed as launch arguments so they can be
overridden from the command line, e.g.:

  ros2 launch wfd_explorer wfd_explorer.launch.py  \\
      map_topic:=/my_map  robot_frame:=base_footprint  \\
      wfd.lambda:=0.7  nav_goal_timeout_s:=90.0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("wfd_explorer")
    default_config = PathJoinSubstitution([pkg_share, "config", "wfd_explorer.yaml"])

    # ── Declare every tuneable as a launch argument ──────────────────────

    declared_args = [
        # Config file (can be swapped entirely)
        DeclareLaunchArgument(
            "config_file",
            default_value=default_config,
            description="Path to the parameter YAML file",
        ),

        # Topics / frames
        DeclareLaunchArgument("map_topic",    default_value="/map",            description="Occupancy grid topic"),
        DeclareLaunchArgument("robot_frame",  default_value="base_link",       description="Robot TF frame"),
        DeclareLaunchArgument("nav2_action",  default_value="navigate_to_pose",description="Nav2 action server name"),

        # Timing
        DeclareLaunchArgument("loop_rate_hz",       default_value="1.0",  description="Exploration loop rate (Hz)"),
        DeclareLaunchArgument("map_timeout_s",       default_value="5.0",  description="Map wait timeout (s)"),
        DeclareLaunchArgument("nav_goal_timeout_s",  default_value="60.0", description="Navigation goal timeout (s)"),
        DeclareLaunchArgument("tf_timeout_s",        default_value="1.0",  description="TF lookup timeout (s)"),

        # Visualisation
        DeclareLaunchArgument("publish_markers", default_value="true",       description="Publish RViz frontier markers"),
        DeclareLaunchArgument("marker_topic",    default_value="~/frontiers", description="Marker array topic"),

        # WFD – thresholding
        DeclareLaunchArgument("wfd.free_threshold",          default_value="50",  description="Occupancy value below which a cell is free"),
        DeclareLaunchArgument("wfd.occ_threshold",           default_value="65",  description="Occupancy value at/above which a cell is an obstacle"),

        # WFD – frontier filtering
        DeclareLaunchArgument("wfd.min_frontier_size",       default_value="3",   description="Minimum frontier size in cells"),
        DeclareLaunchArgument("wfd.max_frontier_split_size", default_value="1.5", description="Max frontier extent (m) before splitting"),

        # WFD – goal selection
        DeclareLaunchArgument("wfd.lambda",             default_value="0.5", description="Info-gain weight (0=nearest, 1=largest)"),
        DeclareLaunchArgument("wfd.info_gain_exponent", default_value="1.0", description="Exponent applied to normalised info gain"),
    ]

    # ── Node ─────────────────────────────────────────────────────────────
    node = Node(
        package="wfd_explorer",
        executable="wfd_explorer_node",
        name="wfd_explorer",
        output="screen",
        emulate_tty=True,
        parameters=[
            # Load the YAML file first (provides defaults)
            LaunchConfiguration("config_file"),
            # Then override with launch-argument values
            {
                "map_topic":              LaunchConfiguration("map_topic"),
                "robot_frame":            LaunchConfiguration("robot_frame"),
                "nav2_action":            LaunchConfiguration("nav2_action"),
                "loop_rate_hz":           LaunchConfiguration("loop_rate_hz"),
                "map_timeout_s":          LaunchConfiguration("map_timeout_s"),
                "nav_goal_timeout_s":     LaunchConfiguration("nav_goal_timeout_s"),
                "tf_timeout_s":           LaunchConfiguration("tf_timeout_s"),
                "publish_markers":        LaunchConfiguration("publish_markers"),
                "marker_topic":           LaunchConfiguration("marker_topic"),
                "wfd.free_threshold":     LaunchConfiguration("wfd.free_threshold"),
                "wfd.occ_threshold":      LaunchConfiguration("wfd.occ_threshold"),
                "wfd.min_frontier_size":  LaunchConfiguration("wfd.min_frontier_size"),
                "wfd.max_frontier_split_size": LaunchConfiguration("wfd.max_frontier_split_size"),
                "wfd.lambda":             LaunchConfiguration("wfd.lambda"),
                "wfd.info_gain_exponent": LaunchConfiguration("wfd.info_gain_exponent"),
            },
        ],
    )

    return LaunchDescription(declared_args + [node])
