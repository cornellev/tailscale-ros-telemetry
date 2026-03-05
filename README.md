# tailscale-ros-telemetry

reading shm telemetry data and publishing as ROS2 topics

## how it works

- the host writes telemetry data to a shared memory
- we read the telemetry data from shared memory and publish it as the ROS2 topic `/spi_data`
- the ROS2 node communicates over a Tailscale network, allowing subscribers to receive data as well

## project structure

```
docker-compose.yml      # service definitions
config/                 # env files and XML profiles
docker/                 # Dockerfiles and entrypoint scripts
src/                    # ROS2 package source
```

## setup

create a `.env` file in the project root with your Tailscale OAuth credentials:

```sh
# required variables:
TS_CLIENT_ID=<your-client-id>
TS_CLIENT_SECRET=<your-client-secret>

# optional variables:

# default = /dev/gpiomem
# for raspberry pi 5, use: /dev/gpiomem4
GPIO_DEVICE=<path-to-gpio-device>

# default = 14
ROS_DOMAIN_ID=<ros-domain-id>
```

## docker compose services

there are four services defined in [docker-compose.yml](docker-compose.yml):

### `tailscale`

Connects the device to your Tailscale network and gates startup for the other services

- authenticates using OAuth credentials (`TS_CLIENT_ID` / `TS_CLIENT_SECRET`) and advertises the tag `tag:ros-device`
- runs [entrypoint-tailscale.sh](docker/entrypoint-tailscale.sh), which waits for the Tailscale connection to be established before exiting the startup loop

### `discovery-server`

Runs a [Fast DDS Discovery Server](https://docs.ros.org/en/humble/Tutorials/Advanced/Discovery-Server/Discovery-Server.html), which centralizes DDS peer discovery instead of using multicast/unicast to all known peers.

- runs [entrypoint-discovery.sh](docker/entrypoint-discovery.sh): sources the ROS2 environment and starts `fastdds discovery --server-id 0` on port 11811 (UDP)
- shares the `tailscale` network namespace, so it is reachable on the Tailscale IP of this device at port 11811
- `ros` connects as a plain CLIENT via `ROS_DISCOVERY_SERVER=127.0.0.1:11811`
- `rosbag` connects as a SUPER_CLIENT via an XML profile

### `ros`

The main ROS2 publisher. Reads telemetry from host shared memory and publishes to the `/spi_data` topic.

- runs [entrypoint-ros.sh](docker/entrypoint-ros.sh): sources the ROS2 environment and launches the `py_pubsub talker` node
- runs as a plain DDS CLIENT via `ROS_DISCOVERY_SERVER=127.0.0.1:11811`
- binds and mounts `/dev/shm` from the host to:
    - read telemetry data from host
    - communicate with `rosbag`, as FastDDS uses shared memory when possible
- accesses GPIO via the configured `GPIO_DEVICE` (default `/dev/gpiomem`)

### `rosbag`

Records the `/spi_data` topic to timestamped bag files.

- runs [entrypoint-rosbag.sh](docker/entrypoint-rosbag.sh): sources the ROS2 environment and runs `ros2 bag record /spi_data`
- runs as a SUPER_CLIENT — required for `ros2 bag record` to discover topic types
- binds and mounts `/dev/shm` from the host for FastDDS shared memory transport with `ros`
- saves bags to `./bags/bag_YYYYMMDD-HHMMSS/`
- starts after the `ros` service has started

## shared memory and discovery

`rosbag` service needs two special things:
- shm
    - fastDDS use shared memory transport for same-host clients, so we need to bind to `/dev/shm` to allow data to flow
- SUPER_CLIENT
    - `ros bag record` requires introspection in order to know the type of a topic. (not possible to manually specify it like you do for `ros topic echo`)
    - with a discovery server, only a SUPER_CLIENT can use introspection

### startup order

the services are started sequentially, one after the other

1. `tailscale`: connects to the tailnet
2. `discovery-server`: starts the Fast DDS discovery server
3. `ros`: starts publishing
4. `rosbag`: starts recording

## How to add a subscriber

Subscribers connect to the discovery server running on this device over the Tailscale network.

The tag is configured via `TAILSCALE_TAG_NAME` on the publisher's `tailscale` service (default: `tag:ros-device`). Subscribers must advertise the same tag.

### CLIENT vs SUPER_CLIENT

FastDDS discovery server supports two client modes:

- **CLIENT** (plain) — set `ROS_DISCOVERY_SERVER=<ip>:11811`. This is sufficient for subscribing to topics you already know the name and type of (e.g. `ros2 topic echo /spi_data std_msgs/msg/String`).
- **SUPER_CLIENT** — required for introspection tools like `ros2 topic list`, `ros2 node list`, etc. Requires an XML profile file pointing at the discovery server.

A `super_client.xml` template is included in this repo. Replace the placeholder IP before use:

```bash
sed 's/DISCOVERY_SERVER_IP/<tailscale-ip-of-publisher>/' super_client.xml > /tmp/super_client.xml
export FASTRTPS_DEFAULT_PROFILES_FILE=/tmp/super_client.xml
```

### Native (non-Docker) subscriber

If you have ROS2 Humble installed natively and your machine is on the same Tailscale network:

```bash
# Set the discovery server (CLIENT mode — enough for topic echo)
export ROS_DISCOVERY_SERVER=<tailscale-ip-of-publisher>:11811

# Subscribe to a known topic
ros2 topic echo /spi_data std_msgs/msg/String

# For introspection tools (ros2 topic list, etc.), use SUPER_CLIENT mode instead:
sed 's/DISCOVERY_SERVER_IP/<tailscale-ip-of-publisher>/' config/super_client.example.xml > /tmp/super_client.xml
export FASTRTPS_DEFAULT_PROFILES_FILE=/tmp/super_client.xml
ros2 topic list
```

### Docker subscriber

Add a Tailscale service to your subscriber's docker-compose using the `tailscale/tailscale:latest` image:

```yaml
  tailscale:
    image: tailscale/tailscale:latest
    # set your container name
    container_name: ts-your-service-authkey-container
    # set your hostname for tailscale
    hostname: your-service-tailscale-service
    environment:
      - TS_CLIENT_ID=${TS_CLIENT_ID:-}
      - TS_CLIENT_SECRET=${TS_CLIENT_SECRET:-}
      - TS_STATE_DIR=/var/lib/tailscale
      - TS_USERSPACE=false
      - TS_AUTH_ONCE=true
      - TS_EXTRA_ARGS=--advertise-tags=tag:ros-device --ssh=true
    volumes:
      # ts-authkey-container is used to persist the auth key state across container restarts
      - ts-authkey-container:/var/lib/tailscale
      # required for tailscale networking to work
      - /dev/net/tun:/dev/net/tun
    cap_add:
      - NET_ADMIN
      - NET_RAW
```

You also need to create the volume for the auth key state:
```yaml
volumes:
  ts-authkey-container:
    driver: local
```

For a subscriber that only needs to echo known topics (CLIENT mode):
```yaml
  your_subscriber_service:
    network_mode: service:tailscale
    environment:
      - ROS_DISCOVERY_SERVER=<tailscale-ip-of-publisher>:11811
```

For a subscriber that needs introspection tools (SUPER_CLIENT mode), generate the XML and mount it:
```bash
sed 's/DISCOVERY_SERVER_IP/<tailscale-ip-of-publisher>/' super_client.xml > /tmp/super_client.xml
```
```yaml
  your_subscriber_service:
    network_mode: service:tailscale
    environment:
      - FASTRTPS_DEFAULT_PROFILES_FILE=/etc/fastdds/super_client.xml
    volumes:
      - /tmp/super_client.xml:/etc/fastdds/super_client.xml:ro
```
