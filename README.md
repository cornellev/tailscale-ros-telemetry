# tailscale-ros

tailscale automation scripts for usage in ROS

`justfile` is just for development purposes.

TODO: this may not be up to date
`launch.sh` is a fully self-contained script that is used to set up tailscale.
requires `curl`, `jq`, and `tailscale` to be installed on the system, but it
will install them if they are not found.

# how to use

## configure env vars
set up these environment variables: (or use direnv or similar)
for nixos use direnv. put the content below into `.envrc` and add `use flake` to the top of it and set the below env vars

```sh
# tailscale oauth client id and secret
export OAUTH_CLIENT_ID=
export OAUTH_CLIENT_SECRET=

# TODO: remove this
# this isn't used directly right now. just set it to anything
export API_KEY=tmp

# TODO: this isn't actually necessary
# set this to the id of your tailnet.
export TAILNET_NAME=

# choose a domain id. must be the same across devices
export ROS_DOMAIN_ID=14

# this must be 0
export ROS_LOCALHOST_ONLY=0

# this cannot be changed either
export RMW_IMPLEMENTATION=rmw_fastrtps_dynamic_cpp

# set this to the full path of the ./fast.xml for you
# TODO: make it done automatically
# TODO: is it possible for to automate populating fast.xml with values by using tailscale?
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/eric/tailscale-ros/fast.xml
```

## set up ros
### nixos

the `flake.nix` sets everything up.
build the ros2 package with `colcon build --symlink-install`

run `source install/setup.zsh` or restart the shell environment, as the devshell does this too

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

run `just full`. this authenticates with tailcale to create an api key,
which then creates an auth key, and then sets up tailscale for this device,
using a tag to ensure the device persists.

TODO: why aren't we using `./launch.sh`?

## run ros

run `ros2 run py_pubsub listener` on one device 
and `ros2 run py_pubsub talker` on the other device.
they should be communicating with each other.

