#!/bin/bash
set -e

# Start tailscale in background
echo "Starting Tailscale containerboot..."
/usr/local/bin/containerboot &
TAILSCALE_PID=$!

# Wait for tailscale to be up
echo "Waiting for Tailscale to connect..."
for i in {1..30}; do
  if tailscale status &>/dev/null; then
    echo "Tailscale is up and connected"
    break
  fi
  if [ $i -eq 30 ]; then
    echo "ERROR: Tailscale failed to connect after 60 seconds"
    exit 1
  fi
  sleep 2
done

# Generate fast.xml
echo "Generating fast.xml..."
if /workspace/launch.sh generate-fast-xml --write /workspace/fast.xml; then
  echo "Successfully generated /workspace/fast.xml"
else
  echo "ERROR: Failed to generate fast.xml"
  exit 1
fi

# Keep container running
echo "Tailscale container ready, keeping alive..."
wait $TAILSCALE_PID
