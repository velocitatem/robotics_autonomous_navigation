#!/usr/bin/env bash
set -euo pipefail

docker compose run --rm rosbot-dev bash -lc "\
  mkdir -p /ws/src/external && \
  vcs import /ws < /ws/rosbot_competition.rosinstall && \
  rm -rf /ws/src/external/rosbot_ros/src/rosbot_navigation && \
  rm -rf /ws/src/external/gazebo_ros_link_attacher"
