#!/usr/bin/env bash

source /opt/ros/humble/setup.bash \
    && source /ros-telemetry/install/setup.bash \
    && export FASTRTPS_DEFAULT_PROFILES_FILE=/workspace/fast.xml \
    && exec ros2 run py_pubsub talker 2>&1 \
    | sed -u 's/^/[talker] /'
