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

TOPIC_NAME="/spi_data"
TIMESTAMP=$(date +'%Y%m%d-%H%M%S')
BAG_DIR="/workspace/bags/bag_${TIMESTAMP}"

echo "Recording topic '${TOPIC_NAME}' to '${BAG_DIR}'..."
exec ros2 bag record "${TOPIC_NAME}" -o "${BAG_DIR}"
