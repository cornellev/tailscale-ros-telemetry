# ros-telemetry

reading shm telemetry data and publishing as ROS2 topics

## how it works

- telemetry data is written to shared memory
- this package reads the telemetry data from shared memory and publishes it a ROS2 topic


## How to add a subscriber

Use the `tailscale/tailscale:latest` container with docker-compose:

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
      # set your tags here. needs to be the same one set by your oauth client
      - TS_EXTRA_ARGS=--advertise-tags=tag:ros-device --ssh=true
      # set this to the exact same tag as above
      - TAILSCALE_TAG_NAME=tag:ros-device
    volumes:
      # ts-authkey-container is used to persist the auth key state across container restarts
      - ts-authkey-container:/var/lib/tailscale
      # required for tailscale networking to work
      - /dev/net/tun:/dev/net/tun
    cap_add:
      - NET_ADMIN
      - NET_RAW

    # subscribers do not need this:
    # for publishers ONLY, we need to build the fast.xml file first before other services can use the service. see entrypoint-tailscale.sh for more info
    # healthcheck: 
    #   test: ["CMD", "test", "-f", "/workspace/fast.xml"]
    #   interval: 2s
    #   timeout: 5s
    #   retries: 30
    #   start_period: 10s
```

You also need to create the volume for the auth key state:
```yaml
volumes:
  ts-authkey-container:
    driver: local
```

For each service that requires connection to tailscale, ensure that it uses the tailscale network:
```yaml
  your_other_service:
    network_mode: service:tailscale
```
