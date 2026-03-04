#!/usr/bin/env bash
# test-discovery.sh
# Verifies that FastDDS discovery server works by:
#   1. Starting a discovery server in Docker
#   2. Starting a publisher connected to it
#   3. Running ros2 topic list and ros2 topic echo as a subscriber
# All containers use the same Docker image (Dockerfile.ros) and an isolated network.

set -e

IMAGE="ros-test-discovery"
NETWORK="test-discovery-net"
DOMAIN_ID=99       # isolated from real traffic
TOPIC="/test_hello"
ECHO_TIMEOUT=10    # seconds to wait for a message

cleanup() {
    echo ""
    echo "==> Cleaning up..."
    docker rm -f test-ds test-pub test-client-echo 2>/dev/null || true
    docker network rm "$NETWORK" 2>/dev/null || true
    rm -f /tmp/super_client_*.xml 2>/dev/null || true
}
trap cleanup EXIT

echo "==> Building ROS image (Dockerfile.ros)..."
docker build -f Dockerfile.ros -t "$IMAGE" . -q

echo "==> Creating isolated Docker network..."
docker network create "$NETWORK" 2>/dev/null || true

# --- Discovery server ---
echo "==> Starting discovery server..."
docker run -d --name test-ds \
    --network "$NETWORK" \
    --entrypoint /entrypoint-discovery.sh \
    "$IMAGE"

echo -n "    Waiting for server to be ready..."
for i in $(seq 1 15); do
    if docker logs test-ds 2>&1 | grep -q "Server is running"; then
        echo " ready."
        break
    fi
    if [ "$i" -eq 15 ]; then
        echo " TIMEOUT"
        echo "Discovery server never became ready. Logs:"
        docker logs test-ds
        exit 1
    fi
    sleep 1
done

# FastDDS does not resolve Docker hostnames for ROS_DISCOVERY_SERVER — use IP directly
DS_IP=$(docker inspect test-ds --format '{{range .NetworkSettings.Networks}}{{.IPAddress}}{{end}}')
echo "    Discovery server IP: $DS_IP"

# --- Generate SUPER_CLIENT XML from template ---
# CLI tools (ros2 topic list, etc.) need SUPER_CLIENT mode to discover
# all topics. Regular CLIENTs only learn about matched endpoints.
SUPER_CLIENT_XML=$(mktemp /tmp/super_client_XXXX.xml)
sed "s/DISCOVERY_SERVER_IP/$DS_IP/" "$(dirname "$0")/super_client.xml" > "$SUPER_CLIENT_XML"

# --- Publisher ---
echo "==> Starting publisher on topic $TOPIC..."
docker run -d --name test-pub \
    --network "$NETWORK" \
    -e ROS_DOMAIN_ID="$DOMAIN_ID" \
    -e ROS_DISCOVERY_SERVER="$DS_IP:11811" \
    --entrypoint bash \
    "$IMAGE" -c \
    "source /opt/ros/humble/setup.bash && ros2 topic pub $TOPIC std_msgs/msg/String '{data: hello_discovery}' --rate 5"

echo "    Waiting 4s for publisher to register with discovery server..."
sleep 4

# --- Test 1: topic list ---
echo "==> Test 1: ros2 topic list (SUPER_CLIENT via discovery server)..."
TOPICS=$(docker run --rm \
    --network "$NETWORK" \
    -e ROS_DOMAIN_ID="$DOMAIN_ID" \
    -e FASTRTPS_DEFAULT_PROFILES_FILE=/tmp/super_client.xml \
    -v "$SUPER_CLIENT_XML":/tmp/super_client.xml:ro \
    --entrypoint bash \
    "$IMAGE" -c \
    "source /opt/ros/humble/setup.bash && sleep 2 && ros2 topic list")

echo "    Topics found: $(echo "$TOPICS" | tr '\n' ' ')"

if echo "$TOPICS" | grep -q "$TOPIC"; then
    echo "    PASS: $TOPIC is visible via discovery server."
else
    echo "    FAIL: $TOPIC not found in topic list."
    echo ""
    echo "Discovery server logs:"
    docker logs test-ds
    echo ""
    echo "Publisher logs:"
    docker logs test-pub
    exit 1
fi

# --- Test 1.5: echo as plain CLIENT (no SUPER_CLIENT XML) ---
# Plain CLIENT mode works for subscribers that know the exact topic name.
# Only introspection tools (ros2 topic list) need SUPER_CLIENT.
echo "==> Test 1.5: ros2 topic echo as plain CLIENT (no SUPER_CLIENT XML)..."
CLIENT_MSG=$(docker run --rm --name test-client-echo \
    --network "$NETWORK" \
    -e ROS_DOMAIN_ID="$DOMAIN_ID" \
    -e ROS_DISCOVERY_SERVER="$DS_IP:11811" \
    --entrypoint bash \
    "$IMAGE" -c \
    "source /opt/ros/humble/setup.bash && timeout $ECHO_TIMEOUT ros2 topic echo --once $TOPIC std_msgs/msg/String 2>/dev/null" || true)

if echo "$CLIENT_MSG" | grep -q "hello_discovery"; then
    echo "    PASS: Plain CLIENT received message: $(echo "$CLIENT_MSG" | grep data)"
else
    echo "    FAIL: Plain CLIENT did not receive a message within ${ECHO_TIMEOUT}s."
    echo "    Output: $CLIENT_MSG"
    echo ""
    echo "Discovery server logs:"
    docker logs test-ds
    exit 1
fi

# --- Test 2: echo a message (SUPER_CLIENT) ---
echo "==> Test 2: ros2 topic echo via SUPER_CLIENT (receive one message)..."
MSG=$(docker run --rm \
    --network "$NETWORK" \
    -e ROS_DOMAIN_ID="$DOMAIN_ID" \
    -e FASTRTPS_DEFAULT_PROFILES_FILE=/tmp/super_client.xml \
    -v "$SUPER_CLIENT_XML":/tmp/super_client.xml:ro \
    --entrypoint bash \
    "$IMAGE" -c \
    "source /opt/ros/humble/setup.bash && timeout $ECHO_TIMEOUT ros2 topic echo --once $TOPIC std_msgs/msg/String 2>/dev/null" || true)

if echo "$MSG" | grep -q "hello_discovery"; then
    echo "    PASS: Message received: $(echo "$MSG" | grep data)"
else
    echo "    FAIL: No message received within ${ECHO_TIMEOUT}s."
    echo "    Output: $MSG"
    echo ""
    echo "Discovery server logs:"
    docker logs test-ds
    exit 1
fi

echo ""
echo "All tests passed. FastDDS discovery server is working correctly."
