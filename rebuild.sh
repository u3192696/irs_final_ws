#!/bin/bash
# Simple ROS2 workspace rebuild helper script

# Exit on error to avoid masking any failures
set -e

# Move to the root of the workspace (use this script's directory)
cd "$(dirname "$0")"

echo ""
echo "=============================="
echo "🚀  Starting colcon build..."
echo "=============================="
echo ""

# Perform the build and stream output
colcon build

echo ""
echo "=============================="
echo "✅  Build complete, sourcing setup..."
echo "=============================="
echo ""

# Source the overlay for this workspace
source install/setup.bash

echo ""
echo "=============================="
echo "🧠  Environment sourced, clearing screen."
echo "=============================="
echo ""

# Clear terminal after short delay so messages are visible
sleep 3
clear

# Optional reminder message
echo "✨ Workspace rebuilt and environment ready. You can now launch:"
echo "   ros2 launch hs_robot_system hs_robot_system.launch.py"
echo ""

