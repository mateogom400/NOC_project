#!/bin/bash

# This script runs a direct, head-to-head comparison between two methods
# (baseline and bo_tuned) for a single, challenging navigation task.

# --- Configuration ---
METHODS=("baseline" "bo_tuned")
PARAMS_BASE_DIR="$(pwd)/src/robot_nav/config"
PLANNER_DELAY_SEC=5.0
GUI="false" # Set to "true" to see the Gazebo GUI

COMPARISON_WORLD="E3_warehouse"
COMPARISON_GOAL="12.0,-2.5,-1.57" # A challenging goal in the warehouse
COMPARISON_BAG_DIR="$(pwd)/bags_recordings/direct_comparison"

# Define initial robot pose for the comparison world
ROBOT_INIT_X="0.0"
ROBOT_INIT_Y="0.0"
ROBOT_INIT_HEADING="0.0"

# --- Script Execution ---

echo "--- Running Direct Comparison ---"
mkdir -p "$COMPARISON_BAG_DIR"

# Source ROS 2 environment
source /opt/ros/humble/setup.bash
if [ -f "install/setup.bash" ]; then
    source "install/setup.bash"
    echo "Sourced local workspace."
else
    echo "Warning: Local workspace not found or not built. Sourcing global ROS 2 only."
fi

world_path="$(pwd)/src/sim_worlds/worlds/${COMPARISON_WORLD}.world"
IFS=',' read -r goal_x goal_y goal_yaw <<< "$COMPARISON_GOAL"

for method in "${METHODS[@]}"; do
    echo "  Running Direct Comparison for Method: '${method}'"
    params_yaml="${PARAMS_BASE_DIR}/${method}_mpc_params.yaml"

    if [ ! -f "$params_yaml" ]; then
        echo "  ERROR: Parameter file not found at ${params_yaml}. Skipping."
        continue
    fi

    BAG_FILE="${COMPARISON_BAG_DIR}/${COMPARISON_WORLD}_${method}_comparison"
    
    echo "  Starting rosbag record. Saving to ${BAG_FILE}"
    ros2 bag record -o "$BAG_FILE" /tf /tf_static /robot_description /scan /odom /cmd_vel /goal_pose /amcl_pose &
    BAG_PID=$!

    # Launch the simulation with a timeout
    timeout 300s ros2 launch robot_sim sim_a_star_mpc.launch.py \
        "gui:=${GUI}" \
        "use_rviz:=${GUI}" \
        "planner_params:=${params_yaml}" \
        "wait_for_goal:=false" \
        "goal_x:=${goal_x}" \
        "goal_y:=${goal_y}" \
        "goal_z:=0.0" \
        "planner_delay_sec:=${PLANNER_DELAY_SEC}" \
        "world:=${world_path}" \
        "world_init_x:=${ROBOT_INIT_X}" \
        "world_init_y:=${ROBOT_INIT_Y}" \
        "world_init_heading:=${ROBOT_INIT_HEADING}"

    echo "  Stopping rosbag record (PID: $BAG_PID)."
    kill -SIGINT $BAG_PID
    sleep 5

    echo "  Cleaning up ROS nodes."
    pkill -f "ros2"
    sleep 5
done

echo "--- Direct Comparison Completed. ---"
