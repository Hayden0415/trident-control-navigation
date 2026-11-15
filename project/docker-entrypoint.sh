#!/bin/bash
set -e
source /opt/ros/humble/setup.bash
source /opt/nav_ws/install/setup.bash
exec "$@"
