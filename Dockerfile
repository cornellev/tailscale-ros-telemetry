FROM ros:humble-ros-core-jammy AS aptgetter

# install ros demos and just
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-demo-nodes-cpp \
    ros-${ROS_DISTRO}-demo-nodes-py \
    just jq \
    && rm -rf /var/lib/apt/lists/*

# install tailscale
RUN apt-get update && apt-get install -y --no-install-recommends ca-certificates curl gnupg \
    && install -m 0755 -d /etc/apt/keyrings \
    && curl -fsSL https://pkgs.tailscale.com/stable/ubuntu/jammy.noarmor.gpg -o /etc/apt/keyrings/tailscale.gpg \
    && echo "deb [signed-by=/etc/apt/keyrings/tailscale.gpg] https://pkgs.tailscale.com/stable/ubuntu jammy main" > /etc/apt/sources.list.d/tailscale.list \
    && apt-get update && apt-get install -y --no-install-recommends tailscale \
    && rm -rf /var/lib/apt/lists/*

# copy files
WORKDIR /app
COPY . .

# source variables, then 
RUN bash -lc 'set -euo pipefail; \
    source .envrc; \
    just generate-api-key; \
    just generate-auth-key'
