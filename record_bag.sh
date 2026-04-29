#!/usr/bin/env bash
# Records essential topics from a running a_star_mpc session.
#
# Usage:
#   ./record_bag.sh [output_dir] [duration_sec]
#
# Example:
#   ./record_bag.sh /tmp/my_run 60
#   ./record_bag.sh                    # defaults below

set -euo pipefail

OUTPUT_DIR="${1:-$(date +%Y%m%d_%H%M%S)_bag}"
DURATION="${2:-}"          # leave empty for unlimited (Ctrl-C to stop)

ROS_SETUP="/opt/ros/jazzy/setup.bash"
if [[ ! -f "$ROS_SETUP" ]]; then
    ROS_SETUP="/opt/ros/humble/setup.bash"
fi

# ROS setup scripts may reference unset variables when this script runs with
# `set -u`. Temporarily disable nounset while sourcing them.
set +u
source "$ROS_SETUP"
set -u

# Try to source workspace setup if available
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_SETUP="$SCRIPT_DIR/../install/setup.bash"
if [[ -f "$WORKSPACE_SETUP" ]]; then
    set +u
    source "$WORKSPACE_SETUP"
    set -u
fi

TOPICS=(
    /odom
    /go2/pose
    /cmd_vel
    /a_star/path
    /mpc/next_setpoint
    /mpc/predicted_path
    /mpc/diagnostics
    /goal_pose
    /tf
    /tf_static
    /lidar/points_filtered
)

mkdir -p "$OUTPUT_DIR"
echo "Recording to: $OUTPUT_DIR"
echo "Topics: ${TOPICS[*]}"
echo "Press Ctrl-C to stop (or bag will auto-stop after ${DURATION:-unlimited} seconds)"

if [[ -n "$DURATION" ]]; then
    timeout --signal INT --kill-after 10s "${DURATION}s" \
        ros2 bag record \
        --output "$OUTPUT_DIR/bag" \
        --storage sqlite3 \
        "${TOPICS[@]}"
else
    ros2 bag record \
        --output "$OUTPUT_DIR/bag" \
        --storage sqlite3 \
        "${TOPICS[@]}"
fi

echo "Bag saved to: $OUTPUT_DIR/bag"
