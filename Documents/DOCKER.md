
# Install and Configure Docker on Raspberry Pi

We will use Docker to manage our ROS environment. Docker enables us to run containers; each container is a separate environment and can contain its own files, and run its own processes. Containers are like lightweight Virtual Machines, without the need to emulate an entire computer.

## Get Docker

Get the install script - From the terminal, type:

```bash
curl -fsSL https://get.docker.com -o get-docker.sh
```
```bash
chmod +x get-docker.sh
 ```

## Remove Docker (If we have old docker installation)

If you want to remove any existing docker installations, do this step before installing docker.

Purge - From the terminal, type:

```bash
sudo apt-get purge docker-ce docker-ce-cli containerd.io -y
 ```

## Install Docker

Run the script - From the terminal, type:

```bash
./get-docker.sh
 ```

## Fix Permissions

Docker can run commands from a regular user account, but first we need to fix the permissions

1. **Make pi user execute docker commands** - From the terminal, type:

```bash
sudo usermod -aG docker pi
 ```
2. **Unmask docker**- From the terminal, type:
 ```bash
sudo systemctl unmask docker
 ```
3. **Fix permissions** - From the terminal, type:
 ```bash
sudo chmod 666 /var/run/docker.sock
 ```
4. **install docker-compose** - From the terminal, type:
 ```bash
pip3 -v install docker-compose
 ```
5. **Start docker** - From the terminal, type:

 ```bash
sudo systemctl start docker
 ```
6. **Reboot** - Restart the Pi to implement the changes. From the terminal, type:

 ```bash
sudo init 6
 ```

 # Creating Dockerfile

The Open Source Robotics Foundation (OSRF) maintain a suite of official Docker container definitions. Lets grab these so that we can build our own container.

**Get official ROS docker images** - https://hub.docker.com/_/ros

For example we can choose - **humble-ros-core-jammy**

```bash
# This is an auto generated Dockerfile for ros:ros-base
# generated from docker_images_ros2/create_ros_image.Dockerfile.em
FROM ros:humble-ros-core-jammy

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
    ros-humble-ros-base=0.10.0-1* \
    && rm -rf /var/lib/apt/lists/*

```

1. Save the above content in a file named **Dockerfile** in your ROS package directory.

2. Build the Docker image

```bash
docker build -t my-ros-container .
```
my-ros-container ---> name of docker container

3. Run the Docker container:

```bash
docker run -it --rm --name my-ros-container my-ros-container
```

4. Check **running status** of Docker Conatiners

```bash
docker ps
```

5. Stop Docker Conatiner

```bash
docker stop <container_name_or_id>

```