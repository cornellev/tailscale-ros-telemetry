# the following for env vars must have already been set, ie with direnv

# oauth client id and secret. make sure not to delete this with delete-key
# possible minimum scopes required: devices, auth keys (read+write)
oauth_client_id := env('OAUTH_CLIENT_ID')
oauth_client_secret := env('OAUTH_CLIENT_SECRET')

# used in the url paths. technically can be replaced with '-', but we use it here for verbosity
tailnet_name := env('TAILNET_NAME')


# api key for the tailnet
# api_key := env('API_KEY')

# when creating a device:
# suppose we are on the new device
# we are given the oauth client id/secret
# we use that to generate an api key
# and then we generate an auth key with that api key
# finally, we use this auth key to connect the device to the tailnet

# the api key can have a short lifetime
# the auth key can also have a short lifetime?

# we need to implement checks to make sure these env vars are set
full:
    API_KEY=$(just generate-api-key | jq -r '.access_token') \
    && echo $API_KEY \
    && AUTH_KEY=$(just generate-auth-key $API_KEY | jq -r '.key') \
    && echo $AUTH_KEY \
    && sudo tailscale up --auth-key=$AUTH_KEY --hostname="$(hostname)-$(date +%s)"

[private]
default:
    @just --list --unsorted

# run an ephemeral tailscale docker container, ie to create a device
[group("docker")]
@run-ephemeral-shell:
    docker run \
        -it \
        --rm \
        --privileged \
        --network host \
        tailscale/tailscale:latest \
        /bin/sh

# creates a new device in docker with the given auth key
[group("docker")]
@make-device auth_key:
    docker run -d \
        --name=tailscaled \
        -v /var/lib:/var/lib \
        -v /dev/net/tun:/dev/net/tun \
        --network=host \
        --cap-add=NET_ADMIN \
        --cap-add=NET_RAW \
        --env TS_AUTHKEY={{auth_key}} \
        tailscale/tailscale

# generates an api key
[group("client")]
@generate-api-key:
    curl -s "https://api.tailscale.com/api/v2/oauth/token" \
        -d "client_id=${OAUTH_CLIENT_ID}" \
        -d "client_secret=${OAUTH_CLIENT_SECRET}" \
            | jq

# lists all devices
[group("api")]
@list-devices api_key:
    curl -s 'https://api.tailscale.com/api/v2/tailnet/{{tailnet_name}}/devices' \
        --header "Authorization: Bearer {{api_key}}" \
            | jq

# creates an auth key
[group("api")]
@generate-auth-key api_key:
    curl -s 'https://api.tailscale.com/api/v2/tailnet/{{tailnet_name}}/keys' \
        --request POST \
        --header 'Content-Type: application/json' \
        --header "Authorization: Bearer {{api_key}}" \
        --data "$(\cat data/create-auth-key.json)" \
            | jq

# lists all keys (client, auth, api)
[group("api")]
@list-keys api_key:
    curl -s 'https://api.tailscale.com/api/v2/tailnet/{{tailnet_name}}/keys' \
        --header "Authorization: Bearer {{api_key}}" \
            | jq

# deletes an key (client, auth, api)
[group("api")]
@delete-key api_key key_id:
    curl -s "https://api.tailscale.com/api/v2/tailnet/{{tailnet_name}}/keys/{{key_id}}" \
        --request DELETE \
        --header "Authorization: Bearer {{api_key}}" \
        | jq

