#!/bin/bash

# Constants.
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
NOCOLOR='\033[0m'

echo This file creates two docker containers. One for ros1_bridge and the other one to run a test ros1 code.

# Create a network
NETWORK=bridge-network
NETWORK_SUBNET="172.16.0.10/16" # subnet mask allows communication between IP addresses with 172.16.xx.xx (xx = any)
CONTAINER1_ROS_IP="172.16.0.20"
CONTAINER2_ROS_IP="172.16.0.21"
ROS_MASTER_URI="http://${CONTAINER1_ROS_IP}:11311"

kill_matching_containers () {
  echo "Killing any running Docker containers matching '$1'..."
  docker ps -a | grep "$1" | awk '{print $1}' | xargs --no-run-if-empty docker kill
  sleep 1

  # It's possible that the container is not running, but still exists.
  echo "Removing any Docker containers matching '$1'..."
  docker ps -a | grep "$1" | awk '{print $1}' | xargs --no-run-if-empty docker rm
  sleep 1
}

echo "Killing containers"
kill_matching_containers "rosmater"

# Create a network
NETWORK=bridge-network
NETWORK_SUBNET="172.16.0.10/16" # subnet mask allows communication between IP addresses with 172.16.xx.xx (xx = any)
CONTAINER1_ROS_IP="172.16.0.20"
CONTAINER2_ROS_IP="172.16.0.21"
ROS_MASTER_URI="http://${CONTAINER1_ROS_IP}:11311"
docker network create --subnet "${SUBNET}" ${NETWORK}

# Container 1 with ros master
COMMAND1="roscore"
docker run --name rosmaster \
            --network ${NETWORK} \
            -e ROS_IP=${CONTAINER1_ROS_IP} \
            -e ROS_MASTER_IP=${ROS_MASTER_IP} \
            -it ros_bridge \
            ${COMMAND1}