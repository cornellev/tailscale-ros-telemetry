#!/bin/bash

set -e

TOPIC_NAME="spi_data"
BAG_NAME="publisher_data_bag"
PUBLISHER_CMD="ros2 run talker"

echo "Starting publisher..."
$PUBLISHER_CMD &
PUBLISHER_PID=$!

sleep 2

echo "Recording topic $TOPIC_NAME to rosbag..."
ros2 bag record $TOPIC_NAME -o $BAG_NAME &
BAG_PID=$!

trap "echo 'Stopping...'; kill $BAG_PID $PUBLISHER_PID; exit" SIGINT

wait
