#!/bin/bash
source /catkin_ws/devel/setup.bash
echo "Sourced Catkin workspace!"
exec "$@"
