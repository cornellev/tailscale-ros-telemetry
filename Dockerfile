# TODO: this is broken
# get the tailscale binaries
# FROM tailscale/tailscale:stable AS tailscale-stage

# final image
FROM ros:humble-ros-core-jammy

# install misc packages
RUN apt-get update && apt-get install -y \
    jq ca-certificates curl gnupg \
    && rm -rf /var/lib/apt/lists/*

# install tailscale
RUN curl -fsSL https://pkgs.tailscale.com/stable/ubuntu/jammy.noarmor.gpg | tee /usr/share/keyrings/tailscale-archive-keyring.gpg >/dev/null \
    && curl -fsSL https://pkgs.tailscale.com/stable/ubuntu/jammy.tailscale-keyring.list | tee /etc/apt/sources.list.d/tailscale.list \
    && apt-get update \
    && apt-get install -y --no-install-recommends tailscale

# copy app files
WORKDIR /app
COPY . .

# source variables, then generate keys and start listener
RUN bash -lc 'set -euo pipefail; \
    source .envrc; \
    ./launch.sh start && \
    ros2 run py_pubsub listener &'
