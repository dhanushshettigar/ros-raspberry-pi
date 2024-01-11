# Configuring Environment - Docker + ROS

ROS 2 relies on the notion of combining workspaces using the shell environment. “Workspace” is a ROS term for the location on your system where you’re developing with ROS 2. The core ROS 2 workspace is called the underlay. Subsequent local workspaces are called overlays. When developing with ROS 2, you will typically have several workspaces active concurrently.

Combining workspaces makes developing against different versions of ROS 2, or against different sets of packages, easier. It also allows the installation of several ROS 2 distributions (or “distros”, e.g. Dashing and Eloquent) on the same computer and switching between them.

This is accomplished by sourcing setup files every time you open a new shell, or by adding the source command to your shell startup script once. Without sourcing the setup files, you won’t be able to access ROS 2 commands, or find or use ROS 2 packages. In other words, you won’t be able to use ROS 2.

## Run ROS docker container

Navigate to the root of your ros workspace and run docker compose - From the terminal, type:

```bash
docker compose up -d
```
The container will now be running, you can check to see if its running using the docker ps command.

## Opening Terminal of docker container

Get container id - From the terminal, type:

```bash
docker ps
```

Open new Terminal and type :

```bash
docker exec -it <container_id> /bin/bash
```

## Source the setup files

You will need to run this command on every new shell you open to have access to the ROS 2 commands

```bash
source /opt/ros/iron/setup.bash
```

## Install turtlesim

Install the turtlesim package for your ROS 2 distro:

```bash
apt update

apt install ros-iron-turtlesim
```
Check that the package is installed:

```bash
ros2 pkg executables turtlesim
```
The above command should return a list of turtlesim’s executables:

```bash
turtlesim draw_square
turtlesim mimic
turtlesim turtle_teleop_key
turtlesim turtlesim_node
```

## Start turtlesim

To start turtlesim, enter the following command in your terminal:

```bash
ros2 run turtlesim turtlesim_node
```

The simulator window should appear, with a random turtle in the center.

# GUI applications from a Docker container

Docker containers typically don't have a graphical user interface (GUI) by default. To run applications with GUI inside a Docker container and display them on your local machine, you'll need to set up X server forwarding.

Here's a simplified example using a Docker Compose YAML file:

```bash
version: "3.9"
services:
 ros2:
   build: .
   environment:
    - DISPLAY=${DISPLAY}
   volumes:
    - /home/ros:/home/ros
    - /tmp/.X11-unix:/tmp/.X11-unix
   devices:
    - /dev/ttyUSB0:/dev/ttyUSB0
   tty: true
```
Save the docker-compose.yml file

Before running the Docker Compose file, you need to allow connections to the X server on your local machine. Run the following command on your local machine:

```bash
xhost +local:
```

Now, you can run the Docker Compose file:

```bash
docker compose up -d
```

## Use turtlesim

Open a new terminal and source ROS 2 again.

Now you will run a new node to control the turtle in the first node:

```bash
ros2 run turtlesim turtle_teleop_key
```

At this point you should have three windows open: a terminal running **turtlesim_node**, a terminal running **turtle_teleop_key** and the **turtlesim window**. 

Arrange these windows so that you can see the turtlesim window, but also have the terminal running turtle_teleop_key active so that you can control the turtle in turtlesim.

Use the arrow keys on your keyboard to control the turtle. It will move around the screen, using its attached “pen” to draw the path it followed so far.

You can see the nodes, and their associated topics, services, and actions, using the list subcommands of the respective commands:

```bash
ros2 node list
ros2 topic list
ros2 service list
ros2 action list
```

## The ROS graph

The ROS graph is a network of ROS 2 elements processing data together at the same time. It encompasses all executables and the connections between them if you were to map them all out and visualize them.

## Nodes in ROS 2

Each node in ROS should be responsible for a single, modular purpose, e.g. controlling the wheel motors or publishing the sensor data from a laser range-finder. Each node can send and receive data from other nodes via topics, services, actions, or parameters.

![Nodes in ros](https://docs.ros.org/en/iron/_images/Nodes-TopicandService.gif)

A full robotic system is comprised of many nodes working in concert. In ROS 2, a single executable (C++ program, Python program, etc.) can contain one or more nodes.

## ros2 run

The command ```ros2 run``` launches an executable from a package.

```bash
ros2 run <package_name> <executable_name>
```
## ros2 node list

ros2 node list will show you the names of all running nodes. This is especially useful when you want to interact with a node, or when you have a system running many nodes and need to keep track of them.

Open a new terminal while turtlesim is still running in the other one, and enter the following command:

```bash
ros2 node list
```

### Remapping

Remapping allows you to reassign default node properties, like node name, topic names, service names, etc., to custom values. In the last tutorial, you used remapping on turtle_teleop_key to change the cmd_vel topic and target turtle2.

Now, let’s reassign the name of our /turtlesim node. In a new terminal, run the following command:

```bash
ros2 run turtlesim turtlesim_node --ros-args --remap __node:=my_turtle
```

## ros2 node info

Now that you know the names of your nodes, you can access more information about them with:

```bash
ros2 node info <node_name>
```

To examine your latest node, my_turtle, run the following command:

```bash
ros2 node info /my_turtle
```

ros2 node info returns a list of subscribers, publishers, services, and actions. i.e. the ROS graph connections that interact with that node.

# Understanding topics in ros

ROS 2 breaks complex systems down into many modular nodes. Topics are a vital element of the ROS graph that act as a bus for nodes to exchange messages.

![topic1 in ros](https://docs.ros.org/en/iron/_images/Topic-SinglePublisherandSingleSubscriber.gif)

A node may publish data to any number of topics and simultaneously have subscriptions to any number of topics.

![topic2 in ros](https://docs.ros.org/en/iron/_images/Topic-MultiplePublisherandMultipleSubscriber.gif)

Topics are one of the main ways in which data is moved between nodes and therefore between different parts of the system.

# Understanding services

Services are another method of communication for nodes in the ROS graph. Services are based on a call-and-response model versus the publisher-subscriber model of topics. While topics allow nodes to subscribe to data streams and get continual updates, services only provide data when they are specifically called by a client.

![services1 in ros](https://docs.ros.org/en/iron/_images/Service-SingleServiceClient.gif)

![services2 in ros](https://docs.ros.org/en/iron/_images/Service-MultipleServiceClient.gif)

# Understanding parameters

A parameter is a configuration value of a node. You can think of parameters as node settings. A node can store parameters as integers, floats, booleans, strings, and lists. In ROS 2, each node maintains its own parameters

# Understanding actions

Actions are one of the communication types in ROS 2 and are intended for long running tasks. They consist of three parts: a goal, feedback, and a result.

Actions are built on topics and services. Their functionality is similar to services, except actions can be canceled. They also provide steady feedback, as opposed to services which return a single response.

Actions use a client-server model, similar to the publisher-subscriber model (described in the topics). An “action client” node sends a goal to an “action server” node that acknowledges the goal and returns a stream of feedback and a result.

![actions in ros](https://docs.ros.org/en/iron/_images/Action-SingleActionClient.gif)