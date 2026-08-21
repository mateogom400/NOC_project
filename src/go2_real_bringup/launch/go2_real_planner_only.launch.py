"""
go2_real_planner_only.launch.py

Brings up only the navigation portion (LiDAR adapter + A*+MPC + goal manager).
Assumes go2_hw_bridge and robot_state_publisher are already running
(e.g. from a previous `ros2 launch go2_bringup go2_hardware.launch.py`).

Useful for restarting the planner without restarting the hardware bridge,
e.g. when swapping between BO-tuned and baseline parameter sets.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():

    bringup_share = get_package_share_directory("go2_real_bringup")
    lidar_share   = get_package_share_directory("g1_real_lidar")
    planner_share = get_package_share_directory("go2_real_planner")
    goal_share    = get_package_share_directory("g1_real_goal_manager")
    safety_share  = get_package_share_directory("robot_safety")

    rviz_config = os.path.join(bringup_share, "rviz", "go2_real.rviz")

    args = [
        DeclareLaunchArgument("use_sim_time",  default_value="false"),
        DeclareLaunchArgument("use_bo_params", default_value="true"),
        DeclareLaunchArgument("params_file",   default_value=""),
        DeclareLaunchArgument("mission_file",  default_value=""),
        DeclareLaunchArgument("use_safety",    default_value="true"),
        DeclareLaunchArgument("use_rviz",      default_value="false"),
    ]

    use_sim_time  = LaunchConfiguration("use_sim_time")
    use_bo_params = LaunchConfiguration("use_bo_params")
    params_file   = LaunchConfiguration("params_file")
    mission_file  = LaunchConfiguration("mission_file")
    use_safety    = LaunchConfiguration("use_safety")
    use_rviz      = LaunchConfiguration("use_rviz")

    mpc_output_topic = PythonExpression(
        ["'/cmd_vel_raw' if '", use_safety, "' == 'true' else '/cmd_vel'"]
    )

    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(lidar_share, "launch", "lidar.launch.py")
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    planner = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(planner_share, "launch", "planner.launch.py")
        ),
        launch_arguments={
            "use_sim_time":  use_sim_time,
            "use_bo_params": use_bo_params,
            "params_file":   params_file,
            "cmd_vel_topic": mpc_output_topic,
        }.items(),
    )

    safety = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(safety_share, "launch", "safety.launch.py")
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
        condition=IfCondition(use_safety),
    )

    goal_manager = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(goal_share, "launch", "goal_manager.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "mission_file": mission_file,
        }.items(),
    )

    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription(args + [lidar, planner, safety, goal_manager, rviz2])
