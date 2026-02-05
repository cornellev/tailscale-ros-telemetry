#!/usr/bin/env bash

source /opt/ros/humble/setup.bash \
    && source /ros-telemetry/install/setup.bash \
    && export FASTRTPS_DEFAULT_PROFILES_FILE=/workspace/fast.xml \
	&& echo "RMW_IMPLEMENTATION is set to: $RMW_IMPLEMENTATION" \
	&& echo "ROS_DOMAIN_ID is set to : $ROS_DOMAIN_ID" \
    && ros2 run py_pubsub talker 2>&1 \
    | sed -u 's/^/[talker] /'
