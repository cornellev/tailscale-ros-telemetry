# get the tailscale binaries
# FROM tailscale/tailscale:stable AS tailscale-stage

# final image
FROM ros:humble-ros-core-jammy

# install misc packages
RUN apt-get update && apt-get install -y \
    just jq ca-certificates curl gnupg \
    && rm -rf /var/lib/apt/lists/*

# install tailscale
RUN curl -fsSL https://pkgs.tailscale.com/stable/ubuntu/jammy.noarmor.gpg | sudo tee /usr/share/keyrings/tailscale-archive-keyring.gpg >/dev/null \
    && curl -fsSL https://pkgs.tailscale.com/stable/ubuntu/jammy.tailscale-keyring.list | sudo tee /etc/apt/sources.list.d/tailscale.list \
    && apt-get update \
    && apt-get install -y --no-install-recommends tailscale

# copy app files
WORKDIR /app
COPY . .

# system dameon boots up the 
# generate crontask to regenrate auth key as necessary inside container

# source variables, then generate keys and start listener
RUN bash -lc 'set -euo pipefail; \
    source .envrc; \
    export API_KEY=$(just generate-api-key | jq -r .access_token); \
    export AUTH_KEY=$(just generate-auth-key | jq -r .key); \
    source install/setup.bash; \
    ros2 run py_pubsub listener &'
