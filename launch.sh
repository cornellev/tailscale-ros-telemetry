#!/usr/bin/env bash

set -euo pipefail

check_dependencies() {
    # install deps only on ubuntu
    if grep -qi ubuntu /etc/os-release; then
        # install necessary dependencies
        if ! command -v curl &> /dev/null; then
            echo "'curl' not found. Installing..." >&2
            sudo apt-get update && sudo apt-get install -y curl || exit 1
        fi

        if ! command -v jq &> /dev/null; then
            echo "'jq' not found. Installing..." >&2
            sudo apt-get update && sudo apt-get install -y jq || exit 1
        fi

        if ! command -v tailscale &> /dev/null; then
            echo "'tailscale' not found. Installing..." >&2
            curl -fsSL https://tailscale.com/install.sh | sh || exit 1
        fi

        # rmw-fastrtps-cpp is installed by default, but we need the dynamic version
        sudo apt-get install -y ros-humble-rmw-fastrtps-dynamic-cpp
    fi
}

generate_api_key() {
    curl -s "https://api.tailscale.com/api/v2/oauth/token" \
        -d "client_id=${OAUTH_CLIENT_ID}" \
        -d "client_secret=${OAUTH_CLIENT_SECRET}"
}

generate_auth_key() {
    name=$1
    api_key=$2
    curl -s 'https://api.tailscale.com/api/v2/tailnet/-/keys' \
        --request POST \
        --header 'Content-Type: application/json' \
        --header "Authorization: Bearer ${api_key}" \
        --data '{
            "keyType": "auth",
            "description": "device creation key '"${name}"'",
            "expirySeconds": 1440,
            "capabilities": { "devices": { "create": {
                "reusable": false,
                "ephemeral": true,
                "preauthorized": true,
                "tags": [
                    "tag:test-devices"
                ]
            } } }
        }'
}

start() {
    # make sure the env vars are set
    if [ -z "${OAUTH_CLIENT_ID:-}" ] || [ -z "${OAUTH_CLIENT_SECRET:-}" ]; then
        echo "ensure OAUTH_CLIENT_ID and OAUTH_CLIENT_SECRET environment variables are set" >&2
        exit 1
    fi

    # generate api key
    api_key="${API_KEY:-}"
    if [ -z "${api_key:-}" ]; then
        api_json=$(generate_api_key 2> /dev/null) || {
            echo "failed to generate api key" >&2
            exit 1
        }
        api_key=$(echo "$api_json" | jq -r '.access_token')
        if [ -z "$api_key" ]; then
            echo "failed to parse api key" >&2
            echo "$api_json" >&2
            exit 1
        fi
        echo "generated api key"
        if [ "$print_keys" = true ]; then
            echo "API Key: $api_key"
        fi
    else
        echo "using api key from environment"
    fi

    name=$(hostname)

    # generate auth key
    auth_json=$(generate_auth_key "$name" "$api_key" 2> /dev/null) || {
        echo "failed to generate auth key" >&2
        exit 1
    }

    auth_key=$(echo "$auth_json" | jq -r '.key')

    if [ -z "$auth_key" ]; then
        echo "failed to parse auth key" >&2
        echo "$auth_json" >&2
        exit 1
    fi
    echo "generated auth key"
    if [ "$print_keys" = true ]; then
        echo "Auth Key: $auth_key"
    fi

    # start tailscale
    sudo tailscale up --auth-key="$auth_key" --hostname="$name" --accept-dns=true --accept-routes=true

    echo "tailscale started with hostname $name"
}

stop() {
    sudo tailscale down
    sudo tailscale logout
    echo "tailscale stopped and logged out"
}

# initialize variables
command=""
install_dependencies=false
print_keys=false

while [ $# -gt 0 ]; do
    case "$1" in
        --help)
            command=help
            shift
            ;;
        --install-deps)
            install_dependencies=true
            shift
            ;;
        --print-keys)
            print_keys=true
            shift
            ;;
        start)
            command=start
            shift
            ;;
        stop)
            command=stop
            shift
            ;;
        restart)
            command=restart
            shift
            ;;

    esac
done

if [ "$install_dependencies" = true ]; then
    check_dependencies
fi

case "${command}" in
    start)
        # if tailscale is running, error out
        if tailscale status &> /dev/null; then
            echo "error: tailscale is already running" >&2
            exit 1
        fi
        start
        ;;
    stop)
        stop
        ;;
    restart)
        stop
        start
        ;;
    *)
        echo "Usage: $0 [options] <command>"
        echo
        echo "Commands:"
        echo "  start              Start tailscale with a new ephemeral device"
        echo "  stop               Stop tailscale and logout"
        echo "  restart            Explicitly stop and start tailscale with a new ephemeral device"
        echo
        echo "Options:"
        echo "  --install-deps     Install necessary dependencies (only on Ubuntu; curl, jq, tailscale)"
        echo "  --print-keys       Print generated API and Auth keys to stdout (for start command)"
        echo "  --help             Show this help message"
        ;;
esac

exit 0
