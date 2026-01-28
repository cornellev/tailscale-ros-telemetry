# tailscale-ros

tailscale automation scripts for usage in ROS

`justfile` is just for development purposes.

TODO: this may not be up to date
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

# path of the fast.xml file. this sets it automatically
export FASTRTPS_DEFAULT_PROFILES_FILE=$(pwd)/fast.xml
```

## set up ros
### nixos

the `flake.nix` sets everything up.

build the ros2 package with `colcon build --symlink-install`

run `source install/setup.zsh`, run `nix develop`, or use direnv to automatically enter the shell.

### other linux

install ros2 humble

setup ros2, `source /opt/ros/humble/setup.zsh`

build the ros2 package with `colcon build --symlink-install`

run `source install/setup.zsh`

## set up fastrtps

in `./fast.xml`, add in all the device ip/hostnames.
this is necessary for the publisher to talk to the subscriber.
We need to have this because tailscale doesn't support multicast,
which ROS uses for the nodes to find each other.

It may be true that the subscribers do not need to have this configured.

TODO: see if subscribers can just not configure fast.xml (makes adding new subscribers much easier)

## set up tailscale

run `./launch.sh start`.
This uses the oauth client to authenticate with tailscale to create an api key,
which then creates an auth key, and then sets up tailscale for this device,
using a tag to ensure the device persists.

TODO: why aren't we using `./launch.sh`?

## run ros

run `ros2 run py_pubsub listener` on one device 
and `ros2 run py_pubsub talker` on the other device.
they should be communicating with each other.

