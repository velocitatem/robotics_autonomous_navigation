#!/usr/bin/env bash
set -e

source /opt/ros/noetic/setup.bash

if [ -f /ws/devel/setup.bash ]; then
  source /ws/devel/setup.bash
fi

exec "$@"
