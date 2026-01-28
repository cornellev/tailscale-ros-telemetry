# tailscale-ros

tailscale automation scripts for usage in ROS

The `justfile` is just for development purposes.

`launch.sh` is a fully self-contained script that is used to set up tailscale.
requires `curl`, `jq`, and `tailscale` to be installed on the system, but it
will install them if they are not found.

# how to use

## configure tailscale

create a tailscale oauth app to get a client ID and secret: 
https://tailscale.com/kb/1215/oauth-clients

Then, create a tag for the devices to use. This repo uses `tag:test-devices`.

TODO: when done developing, change tag to `tag:ros-devices`

## configure env vars
set up these environment variables: (or use direnv or similar)

### nixos
use direnv. put the content below into `.envrc` and add `use flake` to the top of the file.

### other linux

set the following variables:

```sh
# tailscale oauth client id and secret
export OAUTH_CLIENT_ID=
export OAUTH_CLIENT_SECRET=

# choose a domain id. must be the same across devices
export ROS_DOMAIN_ID=14

# this must be 0
export ROS_LOCALHOST_ONLY=0

# this cannot be changed either
export RMW_IMPLEMENTATION=rmw_fastrtps_dynamic_cpp

# path to the fast.xml file. this sets it automatically.
# you can set it to whatever you want, but this is convenient
export FASTRTPS_DEFAULT_PROFILES_FILE=$(pwd)/fast.xml
```

## set up fastrtps

in `./fast.xml`, add in all the device ip/hostnames.
this is necessary for the publisher to talk to the subscriber.
We need to have this because tailscale doesn't support multicast,
which ROS uses for the nodes to find each other.

You can do this automatically with `./launch.sh generate-fast-xml --write fast.xml`

The subscribers do not need to have this configured, only the publishers.

## set up tailscale

If using a `.env` file, make sure to source it first with `source .env`

Run `./launch.sh start`. Add the `--print-keys` flag if you wish to see the generated api and auth keys.

This uses the oauth client to authenticate with tailscale to create an api key,
which then creates an auth key, and then sets up tailscale for this device,
using a tag to ensure the device persists.

## set up ros

these are the standard steps.

### nixos

the `flake.nix` sets everything up.

build the ros2 package with `colcon build --symlink-install`

run `source install/setup.zsh`, run `nix develop`, or use direnv to automatically enter the shell.

### other linux

install ros2 humble

setup ros2, `source /opt/ros/humble/setup.zsh`

build the ros2 package with `colcon build --symlink-install`

run `source install/setup.zsh`

## run ros

run `ros2 run py_pubsub listener` on one device 
and `ros2 run py_pubsub talker` on the other device.
they should be communicating with each other.


# notes

references:
- https://danaukes.com/notebook/ros2/30-configuring-ros-over-tailscale/
- https://github.com/tailscale/tailscale/issues/11972
- https://kamathrobotics.com/ros-2-and-vpns
