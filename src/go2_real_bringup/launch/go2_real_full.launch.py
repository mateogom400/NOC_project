"""
go2_real_full.launch.py

Master launch for the A*+MPC navigation stack on the real Unitree Go2,
driving the robot through its built-in Sport-mode controller (no CHAMP).

Pipeline
--------
  /utlidar/cloud        (Unitree L1)
        │
        ▼   g1_real_lidar.lidar_filter_node
  /lidar/points_filtered
        │                                  ┌── /global_goal ── g1_real_goal_manager
        ▼                                  ▼
  a_star_mpc_planner  (a_star_node + mpc_node + setpoint_to_cmd_vel_node + nav_graph_node)
        │
        ▼  /cmd_vel
  go2_bringup.go2_hw_bridge   ── /api/sport/request ──► Go2 onboard controller
        │
        ▼  /odom, TF(odom→base_footprint)
  back into a_star_mpc_planner

Arguments
---------
  use_bo_params : bool   (default: true)
      true  → load planner_params_bo_tuned.yaml
      false → load planner_params_default.yaml (hand-tuned baseline)
  params_file   : str    (default: "")
      Override path to a planner_params.yaml. When non-empty, takes precedence
      over `use_bo_params`. Use this to load a fresh BO output:
        params_file:=$(ros2 pkg prefix tuning)/.../best_planner_params.yaml
  mission_file  : str    (default: "")
      Optional waypoint-mission YAML. When non-empty, the mission_runner_node
      publishes goals from this file in sequence.
  use_safety    : bool   (default: true)
      Enable robot_safety velocity_limiter between MPC and hw_bridge.
  use_rviz      : bool   (default: true)
  use_sim_time  : bool   (default: false)   keep false on hardware
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():

    bringup_share         = get_package_share_directory("go2_real_bringup")
    go2_bringup_share     = get_package_share_directory("go2_bringup")
    go2_description_share = get_package_share_directory("go2_description")
    lidar_share           = get_package_share_directory("g1_real_lidar")
    planner_share         = get_package_share_directory("go2_real_planner")
    goal_share            = get_package_share_directory("g1_real_goal_manager")
    safety_share          = get_package_share_directory("robot_safety")

    go2_params  = os.path.join(go2_bringup_share,     "config", "go2_params.yaml")
    urdf_file   = os.path.join(go2_description_share, "urdf",   "go2.urdf.xacro")
    rviz_config = os.path.join(bringup_share,         "rviz",   "go2_real.rviz")

    args = [
        DeclareLaunchArgument("use_sim_time",  default_value="false"),
        DeclareLaunchArgument("use_bo_params", default_value="true",
                              description="Load BO-tuned planner params if true, "
                                          "hand-tuned baseline if false"),
        DeclareLaunchArgument("params_file",   default_value="",
                              description="Override planner params file (absolute path). "
                                          "Empty = pick by use_bo_params."),
        DeclareLaunchArgument("mission_file",  default_value="",
                              description="Optional waypoint mission YAML path. "
                                          "Empty = wait for RViz goals only."),
        DeclareLaunchArgument("use_safety",    default_value="true",
                              description="Insert robot_safety velocity_limiter "
                                          "between MPC and hw_bridge"),
        DeclareLaunchArgument("use_rviz",      default_value="true"),
    ]

    use_sim_time  = LaunchConfiguration("use_sim_time")
    use_bo_params = LaunchConfiguration("use_bo_params")
    params_file   = LaunchConfiguration("params_file")
    mission_file  = LaunchConfiguration("mission_file")
    use_safety    = LaunchConfiguration("use_safety")
    use_rviz      = LaunchConfiguration("use_rviz")

    # If use_safety=true, MPC publishes /cmd_vel_raw and the limiter outputs /cmd_vel.
    # Otherwise MPC publishes /cmd_vel directly and hw_bridge consumes it.
    mpc_output_topic = PythonExpression(
        ["'/cmd_vel_raw' if '", use_safety, "' == 'true' else '/cmd_vel'"]
    )

    # ── 1. robot_state_publisher ──────────────────────────────────────────
    robot_description = Command(["xacro ", urdf_file, " use_gazebo:=false"])

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            go2_params,
            {"robot_description": robot_description, "use_sim_time": use_sim_time},
        ],
    )

    # ── 2. Hardware bridge (Unitree Sport API) ────────────────────────────
    # Speaks the Go2 native protocol. Replaces CHAMP entirely.
    go2_hw_bridge = Node(
        package="go2_bringup",
        executable="go2_hw_bridge",
        name="go2_hw_bridge",
        output="screen",
        parameters=[go2_params, {"use_sim_time": use_sim_time}],
    )

    # ── 3. LiDAR adapter ──────────────────────────────────────────────────
    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(lidar_share, "launch", "lidar.launch.py")
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    # ── 4. A* + MPC planner (with BO-YAML selector) ───────────────────────
    planner = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(planner_share, "launch", "planner.launch.py")
        ),
        launch_arguments={
            "use_sim_time":   use_sim_time,
            "use_bo_params":  use_bo_params,
            "params_file":    params_file,
            "cmd_vel_topic":  mpc_output_topic,
        }.items(),
    )

    # ── 5. Safety gate (optional) ─────────────────────────────────────────
    # velocity_limiter: subscribes /cmd_vel_raw  →  publishes /cmd_vel
    safety = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(safety_share, "launch", "safety.launch.py")
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
        condition=IfCondition(use_safety),
    )

    # ── 6. Goal manager (RViz relay + optional mission runner) ────────────
    goal_manager = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(goal_share, "launch", "goal_manager.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "mission_file": mission_file,
        }.items(),
    )

    # ── 7. RViz2 (optional) ───────────────────────────────────────────────
    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription(args + [
        robot_state_publisher,
        go2_hw_bridge,
        lidar,
        planner,
        safety,
        goal_manager,
        rviz2,
    ])
