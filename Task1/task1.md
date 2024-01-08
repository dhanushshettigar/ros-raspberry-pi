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