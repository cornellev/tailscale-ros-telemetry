#!/bin/bash

set -e

TOPIC_NAME="spi_data"
BAG_NAME="publisher_data_bag"
#PUBLISHER_CMD="ros2 run talker"

sleep 2

echo "Recording topic $TOPIC_NAME to rosbag..."
exec ros2 bag record $TOPIC_NAME -o $BAG_NAME 



