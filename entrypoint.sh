#!/bin/bash
source /opt/ros/$ROS_DISTRO/setup.bash
source /catkin_ws/devel/setup.bash
echo "Sourced Catkin workspace!"
exec "$@"
