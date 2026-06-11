"""
Launch file for frontier_exploration.

The launch file lets you swap the active parameter YAML file and toggle
`use_sim_time` from the command line, e.g.:

  ros2 launch frontier_exploration frontier_exploration.launch.py  \\
      config_file:=/path/to/frontier_exploration_husky.yaml  \\
      use_sim_time:=true

The frontier-selection thresholds live in the selected YAML file under
`wfd.min_norm_info_gain` and `wfd.max_norm_occ_deg`, and dead-zone filtering
is configured via `dead_zone_min_distance`.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("frontier_exploration")
    default_config = PathJoinSubstitution([pkg_share, "params", "frontier_exploration_helhest.yaml"])
    elrob_polygon = PathJoinSubstitution([pkg_share, "params", "UTM_Mule_Recon_2026.txt"])

    # ── Declare the launch arguments we expose directly ──────────────────

    declared_args = [
        # Config file (can be swapped entirely)
        DeclareLaunchArgument(
            "config_file",
            default_value=default_config,
            description="Path to the parameter YAML file",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="simulation/bag or not",
        ),
        DeclareLaunchArgument(
            "polygon_file",
            default_value=elrob_polygon,
            description=(
                "Path to an MGRS polygon file (one coordinate per line). "
                "When non-empty this overrides the polygon_file YAML parameter. "
                "The polygon is activated by calling the ~/load_polygon_from_file service."
            ),
        ),
    ]
    # ── Node ─────────────────────────────────────────────────────────────
    node = Node(
        package="frontier_exploration",
        executable="frontier_exploration_node",
        name="frontier_exploration",
        output="screen",
        emulate_tty=True,
        parameters=[
            # Load the YAML file first (provides defaults)
            LaunchConfiguration("config_file"),
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
            {"polygon_file": LaunchConfiguration("polygon_file")},
        ],
    )

    return LaunchDescription(declared_args + [node])
