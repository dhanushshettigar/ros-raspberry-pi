# Interfacing RPlidar A1 with Ros2 Container

LIDAR is one of the fundamental sensing technologies of autonomous vehicles. It gives a system the ability to see how far away things are 360 degrees around it.

## Setting up the docker environment

Navigate to your host workspace where you have **Dockerfile** and **docker-compose.yml** file.

**Step 1** Change the contents of the `Dockerfile` as shown here and save the file

```bash
sudo nano Dockerfile
```

```bash
# This is an auto generated Dockerfile for ros:ros-base
# generated from docker_images_ros2/create_ros_image.Dockerfile.em
FROM ros:iron-ros-core-jammy

# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    build-essential \
    git \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-rosdep \
    python3-vcstool \
    && rm -rf /var/lib/apt/lists/*

# bootstrap rosdep
RUN rosdep init && \
  rosdep update --rosdistro $ROS_DISTRO

# setup colcon mixin and metadata
RUN colcon mixin add default \
      https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml && \
    colcon mixin update && \
    colcon metadata add default \
      https://raw.githubusercontent.com/colcon/colcon-metadata-repository/master/index.yaml && \
    colcon metadata update

# install ros2 packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-iron-ros-base=0.10.0-3* \
    && rm -rf /var/lib/apt/lists/*

# install packages require by us
RUN apt-get update && apt-get install -y --no-install-recommends \
    nano \
    python3-serial \
    ros-iron-rviz2 \
    libogre-1.12.dev \
    && rm -rf /var/lib/apt/lists/*

```

**Step 2** Change the contents of the `docker-compose.yml` as shown here and save the file

```bash
sudo nano docker-compose.yml
```

```bash
version: "3.9"
services:
 ros2:
   build: .
   network_mode: host
   environment:
    - DISPLAY=${DISPLAY}
   volumes:
    - /home/ros:/home/ros
    - /tmp/.X11-unix:/tmp/.X11-unix
   devices:
    - /dev/dri:/dev/dri
    - /dev/ttyUSB0:/dev/ttyUSB0
    - /dev/ttyACM0:/dev/ttyACM0
   tty: true
```

**Step 3** Rebuild the docker image

```bash
docker compose build --no-cache
```
**Step 4** Run the docker image

```bash
docker compose up -d
```
**Step 5** Copy the Container ID

```bash
docker ps
```
**Step 6** Open Container Terminal

*Replace `<container_id>` with copied container id

```bash
docker exec -it <container_id> /bin/bash
```

## Cloning RPlidar package in ros environment

**Step 7** Source the setup files

```bash
source /opt/ros/iron/setup.bash
```

**Step 8** Navigate to ros workspace source directory

```bash
cd home/ros/ros2_ws/src
```

**Step 9** Clone sllidar_ros2 package from Github

Ensure you're still in the ros2_ws/src directory before you clone:

```bash
git clone https://github.com/Slamtec/sllidar_ros2.git
```

**Step 10** Build sllidar_ros2 package

From the root of your workspace (ros2_ws), you can now build sllidar_ros2 package using the command:

```bash
cd ..
colcon build --symlink-install
```

**Step 11** Package environment setup

```bash
source install/setup.bash
```

## Run sllidar_ros2 node

The command for RPLIDAR A1 is :

```bash
ros2 launch sllidar_ros2 sllidar_a1_launch.py
```

## Print Scanner Values

**Step 13** Open a new terminal:

*Replace `<container_id>` with copied container id

```bash
docker exec -it <container_id> /bin/bash
```

**Step 14** Source the setup files
```bash
source /opt/ros/iron/setup.bash
```

**Step 15** Echo the topic data

*run `ros2 topic list` to get publishing topics
```bash
ros2 topic echo /scan
```