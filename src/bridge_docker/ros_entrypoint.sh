#!/bin/bash
set -e

# setup environment
source "/opt/ros/noetic/setup.bash"
# source "/ros2_foxy/install/local_setup.bash"

# unset ROS_HOSTNAME
# export ROS_IP=172.17.0.2
# export ROS_MASTER_IP=10.187.121.212
# export ROS_MASTER_URI=http://${ROS_MASTER_IP}:11311/
# ros2 run ros1_bridge dynamic_bridge
                  

exec "$@"
