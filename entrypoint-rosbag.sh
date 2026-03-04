#!/usr/bin/env bash
set -e

source /opt/ros/humble/setup.bash
source /ros-telemetry/install/setup.bash

echo "ROS_DOMAIN_ID is set to: ${ROS_DOMAIN_ID:-<unset>}"

# ensure ros logs show up in container logs reliably
export RCUTILS_LOGGING_USE_STDOUT=1
export RCUTILS_LOGGING_BUFFERED_STREAM=0
export PYTHONUNBUFFERED=1

# ros2 bag record needs SUPER_CLIENT mode to discover topic types from the
# discovery server. Plain CLIENT mode cannot do topic introspection.
SUPER_CLIENT_XML="/tmp/super_client.xml"
sed 's/DISCOVERY_SERVER_IP/127.0.0.1/' /workspace/super_client.xml > "$SUPER_CLIENT_XML"
export FASTRTPS_DEFAULT_PROFILES_FILE="$SUPER_CLIENT_XML"
unset ROS_DISCOVERY_SERVER
echo "Using SUPER_CLIENT profile: $SUPER_CLIENT_XML"

TOPIC_NAME="/spi_data"
TIMESTAMP=$(date +'%Y%m%d-%H%M%S')
BAG_DIR="/workspace/bags/bag_${TIMESTAMP}"

echo "Recording topic '${TOPIC_NAME}' to '${BAG_DIR}'..."
exec ros2 bag record "${TOPIC_NAME}" -o "${BAG_DIR}"
