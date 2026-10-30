#!/bin/bash
# Copies the teleop node into your ROS package and rebuilds.
# Usage: ./3_install_teleop.sh [workspace_path]   (default: ~/trinh/ag)
set -e
WS=${1:-$HOME/trinh/ag}
PKG=$WS/src/fullrobotassem
HERE=$(cd "$(dirname "$0")" && pwd)

[ -d "$PKG" ] || { echo "Package not found: $PKG"; exit 1; }
mkdir -p "$PKG/scripts"
cp "$HERE/scripts/bluetooth_teleop.py" "$PKG/scripts/"
chmod +x "$PKG/scripts/bluetooth_teleop.py"

cd "$WS"
source /opt/ros/jazzy/setup.bash
colcon build --packages-select fullrobotassem --symlink-install
echo
echo "Installed. Run:  source $WS/install/setup.bash"
echo
echo "Make sure these are in your package:"
echo "  CMakeLists.txt : find_package(rclpy REQUIRED)"
echo "                   install(PROGRAMS scripts/bluetooth_teleop.py DESTINATION lib/\${PROJECT_NAME})"
echo "  package.xml    : <exec_depend>rclpy</exec_depend> <exec_depend>geometry_msgs</exec_depend> <exec_depend>python3-serial</exec_depend>"
