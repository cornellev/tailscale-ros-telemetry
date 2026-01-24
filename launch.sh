#!/usr/bin/env bash

set -euo pipefail

app_name=${APP_NAME:-tailscale-ros}

function ensure_command_installed() {
    local command_name=$1
    if ! command -v "$command_name" &> /dev/null; then
        echo "'$command_name' not found. Install or add to PATH." >&2
        exit 1
    fi
}

if ! command -v curl &> /dev/null; then
    echo "'curl' not found. Installing..." >&2
    apt-get update && apt-get install -y curl || exit 1
fi

if ! command -v jq &> /dev/null; then
    echo "'jq' not found. Installing..." >&2
        apt-get update && apt-get install -y jq || exit 1
fi

if ! command -v tailscale &> /dev/null; then
    echo "'tailscale' not found. Installing..." >&2
    curl -fsSL https://tailscale.com/install.sh | sh || exit 1
fi

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

# make sure the env vars are set
if [ -z "${OAUTH_CLIENT_ID:-}" ] || [ -z "${OAUTH_CLIENT_SECRET:-}" ] ; then
    echo "ensure OAUTH_CLIENT_ID and OAUTH_CLIENT_SECRET environment variables are set" >&2
    exit 1
fi

start() {
    # generate api key
    if [ -z "${api_key:-}" ]; then
        api_json=$(generate_api_key 2>/dev/null) || { echo "failed to generate api key" >&2; exit 1; }
        api_key=$(echo "$api_json" | jq -r '.access_token')
        if [ -z "$api_key" ]; then
            echo "failed to parse api key" >&2
            echo "$api_json" >&2
            exit 1
        fi
        echo "generated api key"
        echo $api_key
    fi

    name=$(hostname) # -$(date +%s)

    # generate auth key
    auth_json=$(generate_auth_key "$name" "$api_key" 2>/dev/null) || { echo "failed to generate auth key" >&2; exit 1; }
    auth_key=$(echo "$auth_json" | jq -r '.key')
    if [ -z "$auth_key" ]; then
        echo "failed to parse auth key" >&2
        echo "$auth_json" >&2
        exit 1
    fi

    # start tailscale
    sudo tailscale up --auth-key="$auth_key" --hostname="$name" --accept-dns=true --accept-routes=true 

    echo "tailscale started with hostname $name"
}

case "${1}" in
    start)
        # if tailscale is running, don't do anything
        if tailscale status &> /dev/null; then
            echo "tailscale is already running" >&2
            exit 1
        fi
        start
        ;;
    stop)
        sudo tailscale down
        sudo tailscale logout
        ;;
    *)
        echo "usage: $0 {start|stop}" >&2
        exit 1
        ;;
esac

exit 0
