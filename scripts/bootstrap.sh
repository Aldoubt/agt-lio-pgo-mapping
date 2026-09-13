#!/usr/bin/env bash
# Install pinned source dependencies and build the ROS 2 Humble workspace.
set -euo pipefail

ROOT="$(git -C "$(dirname "${BASH_SOURCE[0]}")/.." rev-parse --show-toplevel)"
WORKSPACE="$(cd "$ROOT/../.." && pwd)"
EXTERNAL="$WORKSPACE/src/external"

if [[ ! -f /opt/ros/humble/setup.bash || ! -d "$WORKSPACE/src" ]]; then
  echo "Expected this repository at <workspace>/src/agt-lio-pgo-mapping on ROS 2 Humble." >&2
  exit 2
fi

sudo apt-get update
sudo apt-get install -y python3-vcstool python3-rosdep ros-humble-gtsam
if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  sudo rosdep init
fi
rosdep update

mkdir -p "$EXTERNAL"
vcs import "$EXTERNAL" < "$ROOT/.repos"

source /opt/ros/humble/setup.bash
# GTSAM is released as ros-humble-gtsam. Livox is built from the pinned source
# above, so neither name is a rosdep key on Humble.
rosdep install --from-paths "$ROOT" "$EXTERNAL" --ignore-src -r -y \
  --skip-keys "GTSAM livox_ros_driver2"

cd "$WORKSPACE"
colcon build --base-paths "$ROOT" "$EXTERNAL" --packages-up-to agt_mapping_bringup
echo
echo "Build complete. Run: source $WORKSPACE/install/setup.bash"
