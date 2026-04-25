#!/usr/bin/env bash
set -euo pipefail

docker compose run --rm rosbot-dev bash -lc "\
  source /opt/ros/noetic/setup.bash && \
  source /ws/devel/setup.bash && \
  rosrun rosbot_simulation generate_aruco_markers.py --output_dir /ws/src/rosbot_simulation/materials/aruco && \
  roslaunch rosbot_competition_bringup competition_sim.launch"
