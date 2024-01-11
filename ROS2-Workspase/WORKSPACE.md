# Create and Set Up a ROS2 Workspace

When we work on ROS projects we create something called a workspace to keep all our own packages in. You can manage this however you want, but it generally makes sense for each project to have its own workspace, which will contain whatever packages are relevant to that project. Note that here we’re talking about packages to be compiled from source (either written ourselves or pulled from somewhere like GitHub), the system-installed packages will be visible to all our workspaces.

If we were to build our code using normal tools, they wouldn’t necessarily know where to find all the ROS components, and more importantly ROS wouldn’t know where to find OUR packages. This is where the build tool colcon comes in.

When we use ```colcon``` to build our workspace, it will take all our source files from a ```src``` directory, build them into a ```build``` directory, and “install” them into an ```install``` directory. 
This isn’t too different from other build tools, however it also creates a ```setup.bash``` file with all the information the system needs to how to find our packages, just like we had for the system installation. 
So every time we open a terminal to run code from that project, we need to source that workspace by running ```source path/to/workspace/setup.bash```.

![file structure](https://articulatedrobotics.xyz/media/assets/posts/ready-for-ros/colcon.png)

## Run docker container from Host computer

Navigate towards your directory consisting of docker-compose.yml file and run : 

```bash
docker compose up -d
```

Copy the container id by running : 

```bash
docker ps
```

Run this command to open ros container terminal : 

```bash
docker exec -it <container_id> /bin/bash
```

## Source the setup files

You will need to run this command on every new shell you open to have access to the ROS 2 commands, like so:

```bash
source /opt/ros/iron/setup.bash
```

## Add sourcing to your shell startup script

If you don’t want to have to source the setup file every time you open a new shell (skipping task 1), then you can add the command to your shell startup script:

```bash
echo "source /opt/ros/iron/setup.bash" >> ~/.bashrc
```

## Create a workspace

First, create a directory (**ros2_ws**) to contain our workspace:

```bash
mkdir -p ~/ros2_ws/src

cd ~/ros2_ws

```

Our workspace, ros2_ws, will be an overlay on top of the existing ROS 2 installation. In general, it is recommended to use an overlay when you plan to iterate on a small number of packages, rather than putting all of your packages into the same workspace.

## Build the workspace

In the root of the workspace, run ```colcon build```.

```bash
colcon build --symlink-install
```

After the build is finished, we should see the **build**, **install**, and **log** directories.