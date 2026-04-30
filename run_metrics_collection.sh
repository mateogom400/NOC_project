#!/bin/bash

# This script automates the process of running navigation experiments
# for a single world and MPC parameter configurations (methods).
# It launches the simulation, sends a series of goals, and records a ROS bag for each trial.

# --- Configuration ---
METHODS=("copy_planner_params" "planner_params") # (baseline, bo_tuned)
BAG_BASE_DIR="$(pwd)/bags_recordings/metrics_data"
PARAMS_BASE_DIR="$(pwd)/src/a_star_mpc_planner/config"
PLANNER_DELAY_SEC=5.0
GUI="true" # Set to "true" to see the Gazebo GUI
GOAL_REACHED_RADIUS=0.5
GOAL_WAIT_TIMEOUT_SEC=120

# --- Single World Setup ---
WORLD_NAME="indoor_office"

# Initial robot pose for this world
ROBOT_INIT_X=0.0
ROBOT_INIT_Y=0.0
ROBOT_INIT_YAW=0.0

# Define goals for this world
# Format: "x,y,yaw"
GOALS=(
    "7.0,6.0,0.0"
    "-7.0,-2.5,-1.57"
)

# --- Script Execution ---

# Ensure the bag directory exists
mkdir -p "$BAG_BASE_DIR"

# Source ROS 2 environment
source /opt/ros/humble/setup.bash
if [ -f "install/setup.bash" ]; then
    source "install/setup.bash"
    echo "Sourced local workspace."
else
    echo "Warning: Local workspace not found or not built. Sourcing global ROS 2 only."
fi

world_path="$(pwd)/src/sim_worlds/worlds/${WORLD_NAME}.world"
world_name="$(basename "$WORLD_NAME" .world)"

wait_for_goal_reached() {
    local goal_x="$1"
    local goal_y="$2"
    local timeout_sec="$3"

    python3 - "$goal_x" "$goal_y" "$timeout_sec" "$GOAL_REACHED_RADIUS" <<'PY'
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped

goal_x = float(sys.argv[1])
goal_y = float(sys.argv[2])
timeout_sec = float(sys.argv[3])
goal_radius = float(sys.argv[4])


class MonitorNode(rclpy.node.Node):
    def __init__(self):
        super().__init__("goal_wait_monitor")
        self.pose = None
        self.create_subscription(PoseStamped, "/go2/pose", self._pose_cb, 10)

    def _pose_cb(self, msg):
        self.pose = msg


def main() -> int:
    rclpy.init()
    node = MonitorNode()
    start = time.time()
    try:
        while rclpy.ok() and (time.time() - start) < timeout_sec:
            rclpy.spin_once(node, timeout_sec=0.2)
            if node.pose is None:
                continue

            x = node.pose.pose.position.x
            y = node.pose.pose.position.y
            dist = math.hypot(x - goal_x, y - goal_y)
            if dist <= goal_radius:
                print(f"reached: pos=({x:.2f},{y:.2f}) goal=({goal_x:.2f},{goal_y:.2f}) dist={dist:.2f}")
                return 0

        if node.pose is None:
            print("timeout: no /go2/pose received")
        else:
            x = node.pose.pose.position.x
            y = node.pose.pose.position.y
            dist = math.hypot(x - goal_x, y - goal_y)
            print(f"timeout: pos=({x:.2f},{y:.2f}) goal=({goal_x:.2f},{goal_y:.2f}) dist={dist:.2f}")
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


raise SystemExit(main())
PY
}

for method in "${METHODS[@]}"; do
    echo "--- Running Experiment: World='${WORLD_NAME}', Method='${method}' ---"
    
    # Pre-method cleanup: ensure no leftover ros2/gazebo/rosbag processes
    echo "  Pre-clean: killing leftover ros2/rosbag/gz processes."
    pkill -f "ros2 bag record" 2>/dev/null || true
    pkill -f "ros2" 2>/dev/null || true
    pkill -f "gzserver" 2>/dev/null || true
    pkill -f "gzclient" 2>/dev/null || true
    # small wait to let processes terminate
    sleep 2
    params_yaml="${PARAMS_BASE_DIR}/${method}.yaml"

    if [ ! -f "$params_yaml" ]; then
        echo "ERROR: Parameter file not found at ${params_yaml}. Skipping."
        continue
    fi

    goal_count=0
    for goal in "${GOALS[@]}"; do
        goal_count=$((goal_count + 1))
        IFS=',' read -r goal_x goal_y goal_yaw <<< "$goal"

        echo "  Running Task #${goal_count} with Goal: (${goal_x}, ${goal_y})"

        # Define bag file name
        BAG_FILE="${BAG_BASE_DIR}/${WORLD_NAME}_${method}_goal_${goal_count}"
        
        # Start rosbag recording
        echo "  Starting rosbag record. Saving to ${BAG_FILE}"
        ros2 bag record -o "$BAG_FILE" /tf /tf_static /robot_description /scan /odom /cmd_vel /goal_pose /amcl_pose &
        BAG_PID=$!

        # Launch the simulation
        # Launch in the background so we can stop it as soon as the goal is reached.
        ros2 launch robot_sim sim_a_star_mpc.launch.py \
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
            "world_init_heading:=${ROBOT_INIT_YAW}" &
        LAUNCH_PID=$!

        # Give Gazebo/planner time to come up, then wait until the robot reaches
        # the goal (or the timeout is hit).
        sleep 5
        if wait_for_goal_reached "$goal_x" "$goal_y" "$GOAL_WAIT_TIMEOUT_SEC"; then
            echo "  Goal reached — restarting from the starting position for the next goal."
        else
            echo "  Goal wait timed out — stopping the current run and continuing."
        fi

        # Stop the rosbag recording
        echo "  Stopping rosbag record (PID: $BAG_PID)."
        # Stop the rosbag recording politely and wait for it to finish
        if kill -0 "$BAG_PID" 2>/dev/null; then
            kill -SIGINT "$BAG_PID" 2>/dev/null || true
            # wait up to 10s for the bag process to exit
            timeout 10s bash -c "wait $BAG_PID" 2>/dev/null || true
        fi

        # Kill the launch and any remaining ROS/Gazebo nodes to ensure a clean start
        # for the next goal run.
        echo "  Cleaning up ROS and Gazebo nodes."
        if kill -0 "$LAUNCH_PID" 2>/dev/null; then
            kill -SIGINT "$LAUNCH_PID" 2>/dev/null || true
            timeout 10s bash -c "wait $LAUNCH_PID" 2>/dev/null || true
        fi
        pkill -f "ros2 bag record" 2>/dev/null || true
        pkill -f "ros2" 2>/dev/null || true
        pkill -f "gzserver" 2>/dev/null || true
        pkill -f "gzclient" 2>/dev/null || true

        # Wait until no ros2 or rosbag or gz processes remain (max 15s)
        for i in {1..15}; do
            if ! pgrep -f "ros2" >/dev/null && ! pgrep -f "ros2 bag" >/dev/null && ! pgrep -f "gzserver" >/dev/null && ! pgrep -f "gzclient" >/dev/null; then
                break
            fi
            sleep 1
        done

        # Kill any remaining ROS nodes to ensure a clean start for the next run
        echo "  Cleaning up ROS nodes."
        pkill -f "ros2"
        sleep 5

    done
done

echo "--- All experiments completed. ---"