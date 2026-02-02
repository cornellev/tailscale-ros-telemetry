#!/bin/bash
set -e

# start tailscale if enabled (default: true)
if [ "${ENABLE_TAILSCALE:-true}" = "true" ]; then
    echo "Starting tailscale..."
    /launch.sh generate-fast-xml --write fast.xml
    /launch.sh start --print-keys
fi

# run supervisord to manage spi_writer and talker
exec /usr/bin/supervisord -c /etc/supervisor/conf.d/supervisord.conf
