#!/usr/bin/env bash
set -euo pipefail

docker compose run --rm rosbot-dev bash -lc "rosdep install --from-paths src --ignore-src -r -y && catkin_make"
