# Bridge communication between ROS 2 and Ignition Transport

This package provides a network bridge which enables the exchange of messages
between ROS 2 and Ignition Transport.

The bridge is currently implemented in C++. At this point there's no support for
service calls. Its support is limited to only the following message types:

| ROS 2 type                     | Ignition Transport type          |
|--------------------------------|:--------------------------------:|
| std_msgs/msg/String            | ignition::msgs::StringMsg        |

## Prerequisites

* ROS 2 [Dashing](https://index.ros.org/doc/ros2/Installation/Dashing)
* Ignition [Blueprint](https://ignitionrobotics.org/docs/blueprint/install)

### Building the bridge from source

Before continuing you should have the prerequisites for building the bridge from
source installed.

1. Create a colcon workspace:

```
# Setup the workspace
mkdir -p ~/bridge_ws/src
cd ~/bridge_ws/src

# Download needed software
git clone https://github.com/osrf/ros2_ign.git
```

2. Build the workspace:

```
# Source ROS distro's setup.bash
source /opt/ros/dashing/setup.bash

# Build and install into workspace
cd ~/bridge_ws/
colcon build
```

## Example 1a: Ignition Transport talker and ROS 2 listener

TODO

## Example 1b: ROS 2 talker and Ignition Transport listener

TODO

## Example 2: Run the bridge and exchange images

TODO

## Example 3: Static bridge

In this example, we're going to run an executable that starts a bidirectional
bridge for a specific topic and message type. We'll use the `static_bridge`
executable that is installed with the bridge.

The example's code can be found under `ros2_ign_bridge/src/static_bridge.cpp`.
In the code, it's possible to see how the bridge is hardcoded to bridge string
messages published on the `/chatter` topic.

Let's give it a try, starting with Ignition -> ROS 2.

On terminal A, start the bridge:

`ros2 run ros2_ign_bridge static_bridge`

On terminal B, we start a ROS 2 listener:

`ros2 topic echo /chatter std_msgs/msg/String`

And terminal C, publish an Ignition message:

`ign topic pub -t /chatter -m ignition.msgs.StringMsg -p 'data:"Hello"'`

At this point, you should see the ROS 2 listener echoing the message.

Now let's try the other way around, ROS 2 -> Ignition.

On terminal D, start an Igntion listener:

`ign topic -e -t /chatter`

And on terminal E, publish a ROS 2 message:

`ros2 topic pub /chatter std_msgs/msg/String 'data: "Hello"' -1`

You should see the Ignition listener echoing the message.
