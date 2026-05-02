#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$SCRIPT_DIR"

METRICS_BASE_DIR="${METRICS_BASE_DIR:-$REPO_ROOT/bags_recordings/metrics_data}"
PARAMS_BASE_DIR="${PARAMS_BASE_DIR:-$REPO_ROOT/src/a_star_mpc_planner/config}"
WORLD_BASE_DIR="${WORLD_BASE_DIR:-$REPO_ROOT/src/sim_worlds/worlds}"
LAUNCH_PACKAGE="${LAUNCH_PACKAGE:-robot_sim}"
LAUNCH_FILE="${LAUNCH_FILE:-sim_a_star_mpc.launch.py}"

GUI="${GUI:-true}"
USE_RVIZ="${USE_RVIZ:-true}"
PLANNER_DELAY_SEC="${PLANNER_DELAY_SEC:-12.0}"
STACK_STARTUP_SEC="${STACK_STARTUP_SEC:-8}"
MISSION_STARTUP_DELAY_SEC="${MISSION_STARTUP_DELAY_SEC:-3.0}"
GOAL_REACHED_RADIUS="${GOAL_REACHED_RADIUS:-0.35}"
GOAL_WAIT_TIMEOUT_SEC="${GOAL_WAIT_TIMEOUT_SEC:-160}"
POST_GOAL_SETTLE_SEC="${POST_GOAL_SETTLE_SEC:-2.0}"
GOAL_REPUBLISH_SEC="${GOAL_REPUBLISH_SEC:-2.0}"
MISSION_STATUS_LOG_SEC="${MISSION_STATUS_LOG_SEC:-10.0}"
POST_RUN_RECORD_SEC="${POST_RUN_RECORD_SEC:-3.0}"
CLEANUP_WAIT_SEC="${CLEANUP_WAIT_SEC:-25}"
FORCE_OVERWRITE="${FORCE_OVERWRITE:-false}"
RUN_EXTRACTION="${RUN_EXTRACTION:-true}"

ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
if [[ ! -f "$ROS_SETUP" && -f /opt/ros/jazzy/setup.bash ]]; then
  ROS_SETUP=/opt/ros/jazzy/setup.bash
fi
if [[ ! -f "$ROS_SETUP" ]]; then
  echo "ERROR: ROS setup file not found. Set ROS_SETUP=/opt/ros/<distro>/setup.bash" >&2
  exit 1
fi

set +u
source "$ROS_SETUP"
set -u
if [[ -f "$REPO_ROOT/install/setup.bash" ]]; then
  set +u
  source "$REPO_ROOT/install/setup.bash"
  set -u
fi
export PYTHONPATH="$REPO_ROOT/src/a_star_mpc_planner:${PYTHONPATH:-}"

mkdir -p "$METRICS_BASE_DIR"
export ROS_LOG_DIR="${ROS_LOG_DIR:-$METRICS_BASE_DIR/ros_logs}"
mkdir -p "$ROS_LOG_DIR"

METHODS=(baseline bo_optimized)
declare -A METHOD_DIR_SUFFIX=(
  [baseline]="baseline"
  [bo_optimized]="bo_opti"
)

WORLDS=(werehouse indoor_office open_world)
declare -A WORLD_FILE=(
  [werehouse]="warehouse.world"
  [indoor_office]="indoor_office.world"
  [open_world]="$REPO_ROOT/src/go2_sim/worlds/default.sdf"
)
declare -A WORLD_DIR_PREFIX=(
  [werehouse]="werehouse"
  [indoor_office]="indoor_office"
  [open_world]="open_world"
)
declare -A WORLD_INIT_X=(
  [werehouse]="0.0"
  [indoor_office]="0.0"
  [open_world]="0.0"
)
declare -A WORLD_INIT_Y=(
  [werehouse]="0.0"
  [indoor_office]="0.0"
  [open_world]="0.0"
)
declare -A WORLD_INIT_YAW=(
  [werehouse]="0.0"
  [indoor_office]="0.0"
  [open_world]="0.0"
)

BAG_TOPICS=(
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

log() {
  echo "[$(date +'%F %T')] $*"
}

stop_pid() {
  local pid="${1:-}"
  if [[ -z "$pid" ]]; then
    return 0
  fi
  if kill -0 "$pid" 2>/dev/null; then
    kill -SIGINT -- "-$pid" 2>/dev/null || true
    kill -SIGINT "$pid" 2>/dev/null || true
    for _ in {1..20}; do
      if ! kill -0 "$pid" 2>/dev/null; then
        return 0
      fi
      sleep 0.5
    done
    kill -SIGTERM -- "-$pid" 2>/dev/null || true
    kill -SIGTERM "$pid" 2>/dev/null || true
    sleep 1
    kill -SIGKILL -- "-$pid" 2>/dev/null || true
    kill -SIGKILL "$pid" 2>/dev/null || true
  fi
}

cleanup_orphans() {
  local -a patterns=(
    "ros2 bag record"
    "mission_commander.py"
    "sim_a_star_mpc.launch.py"
    "sim_champ.launch.py"
    "rviz2"
    "ign gazebo"
    "gz sim"
    "gzserver"
    "gzclient"
    "robot_state_publisher"
    "quadruped_controller_node"
    "state_estimation_node"
    "ekf_node"
    "static_transform_publisher"
    "parameter_bridge"
    "ros_ign_gazebo.*create"
    "controller_manager.*spawner"
    "velocity_limiter"
    "a_star_node"
    "mpc_node"
    "odom_to_pose_node"
    "setpoint_to_cmd_vel_node"
    "nav_graph_node"
    "cloud_self_filter.py"
  )
  local pattern
  local deadline

  log "Cleaning previous ROS/Gazebo/RViz processes before next task."
  for pattern in "${patterns[@]}"; do
    pkill -INT -f "$pattern" 2>/dev/null || true
  done
  sleep 2

  deadline=$((SECONDS + CLEANUP_WAIT_SEC))
  while (( SECONDS < deadline )); do
    local still_running=false
    for pattern in "${patterns[@]}"; do
      if pgrep -f "$pattern" >/dev/null 2>&1; then
        still_running=true
        break
      fi
    done
    if [[ "$still_running" == "false" ]]; then
      return 0
    fi
    sleep 1
  done

  for pattern in "${patterns[@]}"; do
    pkill -TERM -f "$pattern" 2>/dev/null || true
  done
  sleep 2

  for pattern in "${patterns[@]}"; do
    pkill -KILL -f "$pattern" 2>/dev/null || true
  done
  sleep 2
}

handle_interrupt() {
  log "Interrupted. Cleaning ROS/Gazebo/RViz processes before exiting."
  cleanup_orphans
  exit 130
}
trap handle_interrupt INT TERM

run_has_bag_data() {
  local run_dir="$1"
  [[ -f "$run_dir/RUN_COMPLETE" && -f "$run_dir/bag/metadata.yaml" ]] && compgen -G "$run_dir/bag/*.db3" >/dev/null
}

resolve_params_yaml() {
  local method="$1"
  local candidate
  local -a candidates=()

  case "$method" in
    baseline)
      candidates=(
        "${BASELINE_PARAMS_FILE:-}"
        "$PARAMS_BASE_DIR/copy_params_planner.yaml"
        "$PARAMS_BASE_DIR/copy_planner_params.yaml"
      )
      ;;
    bo_optimized)
      candidates=(
        "${BO_PARAMS_FILE:-}"
        "${GP_PARAMS_FILE:-}"
        "$PARAMS_BASE_DIR/params_plenner.yaml"
        "$PARAMS_BASE_DIR/params_planner.yaml"
        "$PARAMS_BASE_DIR/planner_params.yaml"
      )
      ;;
    *)
      return 1
      ;;
  esac

  for candidate in "${candidates[@]}"; do
    if [[ -n "$candidate" && -f "$candidate" ]]; then
      echo "$candidate"
      return 0
    fi
  done

  return 1
}

waypoints_for_world() {
  local world="$1"
  case "$world" in
    werehouse)
      cat <<'WPS'
-13.0,0.0,3.14;-4.0,0.0,0.0;3.5,0.0,0.0;13.0,0.0,0.0
-13.0,8.0,1.57;-4.0,8.0,0.0;4.0,8.0,0.0;13.0,8.0,0.0
13.0,-8.0,-1.57;4.0,-8.0,3.14;-4.0,-8.0,3.14;-13.0,-8.0,3.14
WPS
      ;;
    indoor_office)
      cat <<'WPS'
8.0,0.0,0.0;8.0,-5.5,-1.57;-6.5,-5.5,3.14;-8.0,0.0,1.57
-8.0,3.0,1.57;-1.0,3.0,0.0;1.5,5.5,1.57;8.0,5.5,0.0
7.5,-3.0,-1.57;3.0,-3.0,3.14;0.0,0.0,3.14;-7.5,3.0,1.57
WPS
      ;;
    open_world)
      cat <<'WPS'
2.0,0.0,0.0;4.0,-2.0,-0.7;7.0,-2.0,0.0;7.0,2.0,1.57;4.0,4.5,2.4;0.0,6.0,3.14
-2.0,0.0,3.14;-5.5,2.0,2.6;-6.5,5.5,1.57;0.0,7.0,0.0;6.5,5.5,0.0;8.0,0.0,-1.57;6.0,-4.0,-2.2;-2.0,-6.5,3.14
0.0,2.0,1.57;2.0,5.5,0.8;-2.0,6.5,2.4;-6.5,2.0,3.14;-6.5,-5.0,-1.57;2.0,-7.0,0.0;7.0,-4.0,0.8
WPS
      ;;
    *)
      return 1
      ;;
  esac
}

run_single_task() {
  local world="$1"
  local method="$2"
  local task_idx="$3"
  local waypoints="$4"

  local world_file="${WORLD_FILE[$world]}"
  if [[ "$world_file" != /* ]]; then
    world_file="$WORLD_BASE_DIR/$world_file"
  fi
  local params_yaml
  if ! params_yaml="$(resolve_params_yaml "$method")"; then
    params_yaml=""
  fi
  local method_dir="${WORLD_DIR_PREFIX[$world]}_${METHOD_DIR_SUFFIX[$method]}"
  local run_dir="$METRICS_BASE_DIR/$method_dir/T${task_idx}"
  local bag_dir="$run_dir/bag"

  if [[ ! -f "$world_file" ]]; then
    log "ERROR: world file not found: $world_file"
    return 1
  fi
  if [[ -z "$params_yaml" || ! -f "$params_yaml" ]]; then
    log "ERROR: params file not found for method: $method"
    return 1
  fi

  if [[ -d "$run_dir" && "$FORCE_OVERWRITE" != "true" ]] && run_has_bag_data "$run_dir"; then
    log "SKIP existing run: $run_dir (set FORCE_OVERWRITE=true to replace)"
    return 0
  fi

  cleanup_orphans

  if [[ -d "$run_dir" ]]; then
    if [[ "$FORCE_OVERWRITE" == "true" ]]; then
      log "Removing existing run because FORCE_OVERWRITE=true: $run_dir"
    else
      log "Removing incomplete run without a valid bag: $run_dir"
    fi
    rm -rf "$run_dir"
  fi

  mkdir -p "$run_dir"

  local launch_log="$run_dir/launch.log"
  local bag_log="$run_dir/bag_record.log"
  local mission_log="$run_dir/mission_commander.log"

  log "Run => world=$world method=$method task=T${task_idx}"
  log "Launch => ros2 launch $LAUNCH_PACKAGE $LAUNCH_FILE"
  log "Params => $params_yaml"
  log "Output => $run_dir"

  setsid ros2 launch "$LAUNCH_PACKAGE" "$LAUNCH_FILE" \
    "gui:=${GUI}" \
    "use_rviz:=${USE_RVIZ}" \
    "planner_params:=${params_yaml}" \
    "wait_for_goal:=true" \
    "planner_delay_sec:=${PLANNER_DELAY_SEC}" \
    "world:=${world_file}" \
    "world_init_x:=${WORLD_INIT_X[$world]}" \
    "world_init_y:=${WORLD_INIT_Y[$world]}" \
    "world_init_heading:=${WORLD_INIT_YAW[$world]}" \
    >"$launch_log" 2>&1 &
  local launch_pid=$!

  sleep "$STACK_STARTUP_SEC"
  if ! kill -0 "$launch_pid" 2>/dev/null; then
    log "ERROR: launch exited during startup. See $launch_log"
    cleanup_orphans
    return 1
  fi

  setsid ros2 bag record \
    --output "$bag_dir" \
    --storage sqlite3 \
    "${BAG_TOPICS[@]}" \
    >"$bag_log" 2>&1 &
  local bag_pid=$!

  set +e
  python3 "$REPO_ROOT/tuning/mission_commander.py" \
    "--waypoints=$waypoints" \
    --goal-topic /goal_pose \
    --pose-topic /go2/pose \
    --goal-radius "$GOAL_REACHED_RADIUS" \
    --goal-timeout-sec "$GOAL_WAIT_TIMEOUT_SEC" \
    --startup-delay-sec "$MISSION_STARTUP_DELAY_SEC" \
    --post-goal-settle-sec "$POST_GOAL_SETTLE_SEC" \
    --republish-sec "$GOAL_REPUBLISH_SEC" \
    --status-log-sec "$MISSION_STATUS_LOG_SEC" \
    >"$mission_log" 2>&1
  local mission_rc=$?
  set -e

  sleep "$POST_RUN_RECORD_SEC"

  stop_pid "$bag_pid"
  stop_pid "$launch_pid"

  cleanup_orphans

  if [[ "$mission_rc" -ne 0 ]]; then
    log "WARNING: mission commander failed for $run_dir (rc=$mission_rc)."
  else
    date +'%F %T' >"$run_dir/RUN_COMPLETE"
    log "Completed: $run_dir"
  fi

  return 0
}

main() {
  log "Starting metrics recording campaign (A*+MPC baseline vs bo_optimized)."
  cleanup_orphans

  for world in "${WORLDS[@]}"; do
    local_task_idx=0
    while IFS= read -r wp_line; do
      [[ -z "$wp_line" ]] && continue
      local_task_idx=$((local_task_idx + 1))
      for method in "${METHODS[@]}"; do
        run_single_task "$world" "$method" "$local_task_idx" "$wp_line"
      done
    done < <(waypoints_for_world "$world")
  done

  if [[ "$RUN_EXTRACTION" == "true" ]]; then
    log "Running metrics extraction (including MPC solving time)..."
    python3 "$REPO_ROOT/tuning/extract_metrics_from_bags.py" --repo-root "$REPO_ROOT"
  fi

  log "All recordings completed."
}

main "$@"
