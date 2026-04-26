#!/usr/bin/env bash
# Run a single scenario inside the rosbot-dev container.
#
# Usage:
#   scripts/docker_sim_scenario.sh <seed> [--gui|--no-gui] [--timeout SEC]
#
# Generates a scenario yaml from the seed (writing it under logs/ so it is
# visible inside the container at /ws/logs/...), passes the robot spawn pose
# and the objects config through to the launch file, and tails rosout for
# `MISSION_COMPLETE` (or a configurable timeout). Exits 0 on success and 1
# on timeout / failure so the suite runner can tally results.
set -euo pipefail

SEED="${1:-0}"
shift || true

GUI="false"
TIMEOUT_SEC="240"
while [[ $# -gt 0 ]]; do
  case "$1" in
    --gui)        GUI="true";  shift ;;
    --no-gui)     GUI="false"; shift ;;
    --timeout)    TIMEOUT_SEC="$2"; shift 2 ;;
    *)
      echo "Unknown arg: $1" >&2
      exit 2
      ;;
  esac
done

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

mkdir -p logs
SCENARIO_HOST="logs/scenario_${SEED}.yaml"
SCENARIO_GUEST="/ws/logs/scenario_${SEED}.yaml"
RUN_LOG="logs/run_${SEED}.log"

echo "==> Generating scenario seed=${SEED}"
META=$(python3 scripts/generate_scenario.py --seed "${SEED}" --output "${SCENARIO_HOST}")
echo "${META}" | sed 's/^/  /'

ROBOT_X=$(echo "${META}" | awk -F= '$1=="ROBOT_X"{print $2}')
ROBOT_Y=$(echo "${META}" | awk -F= '$1=="ROBOT_Y"{print $2}')
ROBOT_YAW=$(echo "${META}" | awk -F= '$1=="ROBOT_YAW"{print $2}')

# Make sure no stale rosbot container is still holding the ROS master / Gazebo.
PREV=$(docker ps --filter "ancestor=rosbot-competition:noetic" --format '{{.ID}}' || true)
if [[ -n "${PREV}" ]]; then
  echo "==> Stopping previous rosbot container ${PREV}"
  docker stop "${PREV}" >/dev/null
fi

echo "==> Launching scenario seed=${SEED} (timeout ${TIMEOUT_SEC}s)"
: > "${RUN_LOG}"
( docker compose run --rm rosbot-dev bash -lc "\
    source /opt/ros/noetic/setup.bash && \
    source /ws/devel/setup.bash && \
    rosrun rosbot_simulation generate_aruco_markers.py \
      --output_dir /ws/src/rosbot_simulation/materials/aruco && \
    roslaunch rosbot_competition_bringup competition_sim.launch \
      gui:=${GUI} \
      robot_x:=${ROBOT_X} \
      robot_y:=${ROBOT_Y} \
      robot_yaw:=${ROBOT_YAW} \
      objects_config:=${SCENARIO_GUEST}" \
  > "${RUN_LOG}" 2>&1 ) &
LAUNCH_PID=$!

trap 'kill ${LAUNCH_PID} 2>/dev/null || true; docker ps --filter ancestor=rosbot-competition:noetic --format "{{.ID}}" | xargs -r docker stop >/dev/null 2>&1 || true' EXIT

# Poll the rosout topic from inside the container for the terminal mission state.
START=$(date +%s)
RESULT="TIMEOUT"
while true; do
  NOW=$(date +%s)
  if (( NOW - START > TIMEOUT_SEC )); then
    break
  fi
  CID=$(docker ps --filter "ancestor=rosbot-competition:noetic" --format '{{.ID}}' | head -n 1)
  if [[ -n "${CID}" ]]; then
    if docker exec "${CID}" bash -lc "test -f /root/.ros/log/latest/mission_controller-*.log" >/dev/null 2>&1; then
      LAST=$(docker exec "${CID}" bash -lc "grep -hE 'MISSION_COMPLETE|all delivery attempts' /root/.ros/log/latest/mission_controller-*.log | tail -n 1" 2>/dev/null || true)
      if echo "${LAST}" | grep -q "MISSION_COMPLETE"; then
        RESULT="SUCCESS"
        break
      fi
    fi
  fi
  sleep 3
done

echo "==> Scenario seed=${SEED} result=${RESULT}"

# Pull the per-mission rosout out so it survives the container teardown.
CID=$(docker ps --filter "ancestor=rosbot-competition:noetic" --format '{{.ID}}' | head -n 1)
if [[ -n "${CID}" ]]; then
  docker exec "${CID}" bash -lc "cat /root/.ros/log/latest/mission_controller-*.log" \
    > "logs/mission_${SEED}.log" 2>/dev/null || true
  docker stop "${CID}" >/dev/null 2>&1 || true
fi

[[ "${RESULT}" == "SUCCESS" ]]
