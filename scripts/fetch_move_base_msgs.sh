#!/usr/bin/env bash
# Vendor move_base_msgs (BSD) from ros-planning/navigation_msgs for systems
# that cannot `apt install ros-noetic-move-base-msgs` (e.g. no admin rights).
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
DEST="${ROOT}/src/third_party/move_base_msgs"
TMP="$(mktemp -d)"
cleanup() { rm -rf "$TMP"; }
trap cleanup EXIT
git clone --depth 1 -b ros1 https://github.com/ros-planning/navigation_msgs.git "$TMP/repo"
rm -rf "$DEST"
cp -a "$TMP/repo/move_base_msgs" "$DEST"
echo "Installed move_base_msgs into ${DEST} — rebuild with: catkin_make"
