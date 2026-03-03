# tailscale-ros-telemetry

reading shm telemetry data and publishing as ROS2 topics

## how it works

- the host writes telemetry data to a shared memory
- we read the telemetry data from shared memory and publishes it as the ros ROS2 topic `/spi_data`
- the ROS2 node communicates over a Tailscale network, allowing subscribers to receive data as well

## docker compose services

there are four services defined in [docker-compose.yml](docker-compose.yml):

### `tailscale`

Connects the device to your Tailscale network and gates startup for the other services

- authenticates using OAuth credentials (`TS_CLIENT_ID` / `TS_CLIENT_SECRET`) and advertises the tag set in `TAILSCALE_TAG_NAME` (default: `tag:ros-device`)
- runs [entrypoint-tailscale.sh](entrypoint-tailscale.sh), which waits for the Tailscale connection to be established before exiting the startup loop

### `discovery-server`

Runs a [Fast DDS Discovery Server](https://docs.ros.org/en/humble/Tutorials/Advanced/Discovery-Server/Discovery-Server.html), which centralizes DDS peer discovery instead of using multicast/unicast to all known peers.

- runs [entrypoint-discovery.sh](entrypoint-discovery.sh): sources the ROS2 environment and starts `fastdds discovery --server-id 0` on port 11811 (UDP)
- shares the `tailscale` network namespace, so it is reachable on the Tailscale IP of this device at port 11811
- the `ros` and `rosbag` services connect to it via `ROS_DISCOVERY_SERVER=127.0.0.1:11811`

### `ros`

The main ROS2 publisher. Reads telemetry from host shared memory and publishes to the `/spi_data` topic.

- runs [entrypoint-ros.sh](entrypoint-ros.sh): sources the ROS2 environment and launches the `py_pubsub talker` node
- mounts `/dev/shm` from the host to read shared memory
- accesses GPIO via the configured `GPIO_DEVICE` (default `/dev/gpiomem`)

### `rosbag`

Records the `/spi_data` topic to timestamped bag files.

- Runs [entrypoint-rosbag.sh](entrypoint-rosbag.sh): sources the ROS2 environment and runs `ros2 bag record /spi_data`
- saves bags to `./bags/bag_YYYYMMDD-HHMMSS/`
- starts after the `ros` service has started

### startup order

1. `tailscale`: connects to the tailnet
2. `discovery-server`: starts the Fast DDS discovery server
3. `ros`: starts publishing
4. `rosbag`: starts recording

## How to add a subscriber

Subscribers connect to the discovery server running on this device. Set `ROS_DISCOVERY_SERVER` to the Tailscale IP of this device on port 11811, and the subscriber will automatically discover all ROS2 topics.

The tag is configured via `TAILSCALE_TAG_NAME` on the publisher's `tailscale` service (default: `tag:ros-device`). Subscribers must advertise the same tag.

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
      # must match the TAILSCALE_TAG_NAME set on the publisher (default: tag:ros-device)
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

For each service that needs to be on the Tailscale network, use the `tailscale` network namespace, and point it at the discovery server:
```yaml
  your_subscriber_service:
    network_mode: service:tailscale
    environment:
      - ROS_DISCOVERY_SERVER=<tailscale-ip-of-publisher>:11811
```
