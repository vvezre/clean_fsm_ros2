#!/usr/bin/env bash
set -euo pipefail

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "ROS2 Humble is not installed at /opt/ros/humble" >&2
  exit 1
fi

# Humble's generated setup scripts may inspect variables that are not defined yet.
set +u
source /opt/ros/humble/setup.bash
set -u

if [[ "${ROS_DISTRO:-}" != "humble" ]]; then
  echo "Expected ROS_DISTRO=humble, got ${ROS_DISTRO:-unset}" >&2
  exit 1
fi

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${workspace_dir}"

colcon build \
  --symlink-install \
  --event-handlers console_direct+ \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=ON

colcon test --event-handlers console_direct+
colcon test-result --verbose
