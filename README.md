# tailscale-ros-telemetry

reading shm telemetry data and publishing as ROS2 topics

## how it works

- the host writes telemetry data to a shared memory
- we read the telemetry data from shared memory and publishes it as the ros ROS2 topic `/spi_data`
- the ROS2 node communicates over a Tailscale network, allowing subscribers to receive data as well

## docker compose services

there are three services defined in [docker-compose.yml](docker-compose.yml):

### `tailscale`

Connects the device to your Tailscale network and gates startup for the other services

- authenticates using OAuth credentials (`TS_CLIENT_ID` / `TS_CLIENT_SECRET`) and advertises the tag set in `TAILSCALE_TAG_NAME` (default: `tag:ros-device`)
- after connecting to the tailnet, runs [entrypoint-tailscale.sh](entrypoint-tailscale.sh) which generates `fast.xml` — a FastRTPS profile that configures ROS2 DDS discovery to use the device's Tailscale IP, enabling cross-network ROS2 communication
- exposes a healthcheck that blocks the other services from starting until `fast.xml` has been written

### `ros`

The main ROS2 publisher. Reads telemetry from host shared memory and publishes to the `/spi_data` topic.

- runs [entrypoint-ros.sh](entrypoint-ros.sh): configures the ROS2 environment, uses the fast.xml file, and then launches the `py_pubsub talker` node
- mounts `/dev/shm` from the host to read shared memory
- accesses GPIO via the configured `GPIO_DEVICE` (default `/dev/gpiomem`)

### `rosbag`

Records the `/spi_data` topic to timestamped bag files.

- Runs [entrypoint-rosbag.sh](entrypoint-rosbag.sh): uses the same dockerfile as `ros`, uses fast.xml, but instead, runs `ros2 bag record /spi_data`
- saves bags to `./bags/bag_YYYYMMDD-HHMMSS/`
- starts after the `ros` service has started

### startup order

1. `tailscale`: connects and generates fast.xml
2. `ros`: starts publishing
3. `rosbag`: starts recording

## How to add a subscriber

The publisher discovers subscribers (not the other way around). When the publisher starts, `generate_fast_xml` queries the Tailscale API for all peers with a matching tag and includes them as initial DDS peers in `fast.xml`. So a subscriber just needs to join the tailnet with that tag **before the publisher starts** (or the publisher needs to be restarted to pick it up).
c
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

For each service that needs to be on the Tailscale network, use the `tailscale` network namespace:
```yaml
  your_subscriber_service:
    network_mode: service:tailscale
```
