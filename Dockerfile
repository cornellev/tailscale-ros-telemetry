# final image
FROM ros:humble-ros-core-jammy

# build args
ARG INSTALL_TAILSCALE=true

# install misc packages
RUN apt-get update && apt-get install -y \
    jq ca-certificates curl gnupg git build-essential supervisor \
    && rm -rf /var/lib/apt/lists/*

# install tailscale (optional)
RUN if [ "$INSTALL_TAILSCALE" = "true" ]; then \
    curl -fsSL https://pkgs.tailscale.com/stable/ubuntu/jammy.noarmor.gpg | tee /usr/share/keyrings/tailscale-archive-keyring.gpg >/dev/null \
    && curl -fsSL https://pkgs.tailscale.com/stable/ubuntu/jammy.tailscale-keyring.list | tee /etc/apt/sources.list.d/tailscale.list \
    && apt-get update \
    && apt-get install -y --no-install-recommends tailscale \
    && rm -rf /var/lib/apt/lists/*; \
    fi

# packages for pi gpio
RUN apt-get update && apt-get install -y \
    python3-pip python3-lgpio python3-pigpio python3-rpi.gpio \
    && rm -rf /var/lib/apt/lists/*
RUN pip3 install gpiozero --break-system-packages

# install spi sensor reader
RUN git clone https://github.com/cornellev/spi_sensor_reader /spi_sensor_reader
WORKDIR /spi_sensor_reader
RUN g++ -O2 -std=c++17 spi_shm.cpp \
    -lpigpiod_if2 -lrt -pthread \
    -o spi_writer

# copy and build tailscale-ros-telemetry (this repo)
COPY src/ /tailscale-ros-telemetry/src/
WORKDIR /tailscale-ros-telemetry
RUN source /opt/ros/humble/setup.sh && colcon build --symlink-install

COPY entrypoint.sh /entrypoint.sh
COPY launch.sh /launch.sh
COPY supervisord.conf /etc/supervisor/conf.d/supervisord.conf
RUN chmod +x /entrypoint.sh /launch.sh

ENTRYPOINT ["/entrypoint.sh"]
