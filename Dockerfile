FROM ros:humble-ros-base-jammy

# install packages
RUN apt-get update && apt-get install -y \
    ca-certificates gnupg git build-essential unzip wget ros-humble-rmw-fastrtps-cpp \
    && rm -rf /var/lib/apt/lists/*

# packages for pi gpio
# RUN apt-get update && apt-get install -y \
#     python3-pip python3-lgpio python3-pigpio  \
#     && rm -rf /var/lib/apt/lists/*
# RUN pip3 install gpiozero

# install pigpio from source
RUN git clone https://github.com/joan2937/pigpio /pigpio
WORKDIR /pigpio
RUN make && make install

RUN apt-get update && apt-get install -y \
    cmake libpthread-stubs0-dev  \
    && rm -rf /var/lib/apt/lists/*

# install spi sensor reader
RUN git clone --depth=1 https://github.com/cornellev/spi_sensor_reader /spi_sensor_reader
WORKDIR /spi_sensor_reader
RUN g++ -O2 -std=c++17 spi_shm.cpp \
    -lpigpiod_if2 -lrt -pthread \
    -o spi_writer

# copy and build ros-telemetry (this repo)
COPY src/ /ros-telemetry/src/
WORKDIR /ros-telemetry
RUN . /opt/ros/humble/setup.sh && colcon build --symlink-install

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
