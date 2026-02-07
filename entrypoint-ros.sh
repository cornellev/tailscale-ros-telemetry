#!/usr/bin/env bash
set -e

source /opt/ros/humble/setup.bash
source /ros-telemetry/install/setup.bash
export FASTRTPS_DEFAULT_PROFILES_FILE=/workspace/fast.xml

echo "RMW_IMPLEMENTATION is set to: ${RMW_IMPLEMENTATION:-<unset>}"
echo "ROS_DOMAIN_ID is set to: ${ROS_DOMAIN_ID:-<unset>}"

# ensure ros logs show up in container logs reliably
export RCUTILS_LOGGING_USE_STDOUT=1
export RCUTILS_LOGGING_BUFFERED_STREAM=0
export PYTHONUNBUFFERED=1

echo "Starting talker node..."
exec ros2 run py_pubsub talker
