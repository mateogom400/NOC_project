"""
planner.launch.py

Launches the A*+MPC planner stack from a_star_mpc_planner on the real Go2.

Spawns
------
  odom_to_pose_node          /odom → /go2/pose
  a_star_node                Gaussian grid + A* on /lidar/points_filtered
  mpc_node                   Nonlinear MPC (CasADi/IPOPT) tracker
  setpoint_to_cmd_vel_node   P-controller publishing /cmd_vel (body frame)
  nav_graph_node             Topological global memory (Dijkstra)

Parameter selection
-------------------
  params_file   : str   absolute path; takes precedence over use_bo_params.
                        Use to deploy a fresh tuning output, e.g.:
                          params_file:=/path/to/tuning_results/best_planner_params.yaml
  use_bo_params : bool  true  → planner_params_bo_tuned.yaml (Phase 1)
                        false → planner_params_default.yaml  (Phase 0 baseline)

Other arguments
---------------
  cmd_vel_topic : str   Output topic for setpoint_to_cmd_vel_node.
                        Default '/cmd_vel'. Bringup overrides to '/cmd_vel_raw'
                        when the safety gate is enabled.
  use_sim_time  : bool  Keep false on hardware (default false).
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _resolve_params_file(context, *args, **kwargs):
    """
    Resolve params_file at launch-time:
      1. If params_file arg is non-empty → use it.
      2. Else if use_bo_params == "true" → planner_params_bo_tuned.yaml
      3. Else                            → planner_params_default.yaml
    """
    share = get_package_share_directory("go2_real_planner")
    bundled_bo      = os.path.join(share, "config", "planner_params_bo_tuned.yaml")
    bundled_default = os.path.join(share, "config", "planner_params_default.yaml")

    explicit       = LaunchConfiguration("params_file").perform(context).strip()
    use_bo         = LaunchConfiguration("use_bo_params").perform(context).lower() == "true"
    use_sim_time   = LaunchConfiguration("use_sim_time").perform(context).lower() == "true"
    cmd_vel_topic  = LaunchConfiguration("cmd_vel_topic").perform(context)

    if explicit:
        params_file = explicit
        source = "params_file override"
    elif use_bo:
        params_file = bundled_bo
        source = "bundled BO-tuned"
    else:
        params_file = bundled_default
        source = "bundled hand-tuned baseline"

    if not os.path.isfile(params_file):
        raise FileNotFoundError(
            f"Planner params file not found: {params_file} (source={source})"
        )

    common_params = [params_file, {"use_sim_time": use_sim_time}]

    nodes = [
        Node(
            package="a_star_mpc_planner",
            executable="odom_to_pose_node",
            name="odom_to_pose_node",
            output="screen",
            parameters=common_params,
        ),
        Node(
            package="a_star_mpc_planner",
            executable="a_star_node",
            name="a_star_node",
            output="screen",
            parameters=common_params,
        ),
        Node(
            package="a_star_mpc_planner",
            executable="mpc_node",
            name="mpc_node",
            output="screen",
            parameters=common_params,
        ),
        Node(
            package="a_star_mpc_planner",
            executable="setpoint_to_cmd_vel_node",
            name="setpoint_to_cmd_vel_node",
            output="screen",
            parameters=common_params,
            remappings=[("/cmd_vel", cmd_vel_topic)],
        ),
        Node(
            package="a_star_mpc_planner",
            executable="nav_graph_node",
            name="nav_graph_node",
            output="screen",
            parameters=common_params,
        ),
    ]

    print(f"[go2_real_planner] params source: {source}")
    print(f"[go2_real_planner] params file  : {params_file}")
    print(f"[go2_real_planner] cmd_vel out  : {cmd_vel_topic}")
    return nodes


def generate_launch_description():
    args = [
        DeclareLaunchArgument("use_sim_time",  default_value="false"),
        DeclareLaunchArgument("use_bo_params", default_value="true",
                              description="Load planner_params_bo_tuned.yaml if true, "
                                          "planner_params_default.yaml if false"),
        DeclareLaunchArgument("params_file",   default_value="",
                              description="Override absolute path to a planner_params.yaml; "
                                          "empty = pick by use_bo_params"),
        DeclareLaunchArgument("cmd_vel_topic", default_value="/cmd_vel",
                              description="Output topic for setpoint_to_cmd_vel_node"),
    ]
    return LaunchDescription(args + [OpaqueFunction(function=_resolve_params_file)])
