"""
Launch file for frontier_exploration.

All tunable parameters are exposed as launch arguments so they can be
overridden from the command line, e.g.:

  ros2 launch frontier_exploration frontier_exploration.launch.py  \\
      map_topic:=/my_map  robot_frame:=base_footprint  \\
      wfd.lambda:=0.7  nav_goal_timeout_s:=90.0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("frontier_exploration")
    default_config = PathJoinSubstitution([pkg_share, "params", "frontier_exploration.yaml"])

    # ── Declare every tuneable as a launch argument ──────────────────────

    declared_args = [
        # Config file (can be swapped entirely)
        DeclareLaunchArgument(
            "config_file",
            default_value=default_config,
            description="Path to the parameter YAML file",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="simulation/bag or not",
        )
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
        ],
    )

    return LaunchDescription(declared_args + [node])
