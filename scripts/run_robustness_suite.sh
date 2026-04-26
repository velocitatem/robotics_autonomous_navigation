#!/usr/bin/env bash
# Run the mission across multiple randomised scenarios to expose sim-to-real
# fragility. Each seed yields a different no-tag corner, colour-to-corner
# mapping, robot spawn jitter, and puck layout.
#
# Usage:
#   scripts/run_robustness_suite.sh [seeds...] [--timeout SEC] [--gui|--no-gui]
#
#   With no seed args, defaults to seeds 0 1 2 3.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

SEEDS=()
TIMEOUT_SEC="240"
GUI_FLAG="--no-gui"
while [[ $# -gt 0 ]]; do
  case "$1" in
    --gui)     GUI_FLAG="--gui"; shift ;;
    --no-gui)  GUI_FLAG="--no-gui"; shift ;;
    --timeout) TIMEOUT_SEC="$2"; shift 2 ;;
    -*) echo "Unknown flag $1" >&2; exit 2 ;;
    *)  SEEDS+=("$1"); shift ;;
  esac
done

if [[ ${#SEEDS[@]} -eq 0 ]]; then
  SEEDS=(0 1 2 3)
fi

mkdir -p logs
SUMMARY="logs/robustness_summary.txt"
: > "${SUMMARY}"

echo "Robustness suite: seeds=${SEEDS[*]} timeout=${TIMEOUT_SEC}s gui=${GUI_FLAG}" | tee -a "${SUMMARY}"
echo "---" | tee -a "${SUMMARY}"

PASS=0
FAIL=0
for SEED in "${SEEDS[@]}"; do
  set +e
  scripts/docker_sim_scenario.sh "${SEED}" ${GUI_FLAG} --timeout "${TIMEOUT_SEC}"
  RC=$?
  set -e
  if [[ ${RC} -eq 0 ]]; then
    RESULT="PASS"
    PASS=$((PASS + 1))
  else
    RESULT="FAIL"
    FAIL=$((FAIL + 1))
  fi

  # Pull a one-line summary of mission progress from the per-run log.
  PROGRESS="?"
  if [[ -f "logs/mission_${SEED}.log" ]]; then
    PROGRESS=$(grep -E 'Grabbed|Arrived|MISSION_COMPLETE|all delivery attempts' \
      "logs/mission_${SEED}.log" | tail -n 8 | sed -E 's/^.*\[MISSION\] //' | tr '\n' '|')
  fi

  printf 'seed=%-3s %-4s   %s\n' "${SEED}" "${RESULT}" "${PROGRESS}" \
    | tee -a "${SUMMARY}"
done

echo "---" | tee -a "${SUMMARY}"
echo "Total: ${PASS} pass / ${FAIL} fail (out of ${#SEEDS[@]})" | tee -a "${SUMMARY}"
[[ ${FAIL} -eq 0 ]]
