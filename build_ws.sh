#!/usr/bin/env bash
set -e

# Source the global ROS 2 Humble installation
source /opt/ros/humble/setup.bash

echo "Building Workspace with Colcon..."

# Build the workspace
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# Source the local workspace
source install/setup.bash

echo "Build complete. Workspace sourced."