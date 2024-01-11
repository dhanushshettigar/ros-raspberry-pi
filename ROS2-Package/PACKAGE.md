# Creating a ros2 package

A package is an organizational unit for your ROS 2 code. If you want to be able to install your code or share it with others, then you’ll need it organized in a package. With packages, you can release your ROS 2 work and allow others to build and use it easily.

Package creation in ROS 2 uses ament as its build system and colcon as its build tool. You can create a package using either CMake or Python, which are officially supported, though other build types do exist.

## What makes up a ROS 2 package?

ROS 2 Python and CMake packages each have their own minimum required contents

For Python

* **```package.xml```** file containing meta information about the package

* **```resource/<package_name>```** marker file for the package

* **```setup.cfg```** is required when a package has executables, so ros2 run can find them

* **```setup.py```** containing instructions for how to install the package

* **```<package_name>```** - a directory with the same name as your package, used by ROS 2 tools to find your package, contains __init__.py

A single workspace can contain as many packages as you want, each in their own folder. You can also have packages of different build types in one workspace (CMake, Python, etc.).

Best practice is to have a src folder within your workspace, and to create your packages in there. This keeps the top level of the workspace “clean”.

## Creating the package

Let’s use the workspace you created **ros2_ws** for our new package.

Make sure you are in the src folder before running the package creation command.

The command syntax for creating a new package in ROS 2 is:

```bash
ros2 pkg create --build-type ament_python <package_name>
```

For this task, you will use the optional arguments __--node-name__. *--node-name* option creates a simple Hello World type executable in the package.

```bash
ros2 pkg create --build-type ament_python --node-name my_node my_package
```
## Build a package

Putting packages in a workspace is especially valuable because you can build many packages at once by running colcon build in the workspace root. Otherwise, you would have to build each package individually.

Return to the root of your workspace:

```bash
cd ~/ros2_ws
```

Now you can build your packages:

```bash
colcon build --packages-select my_package
```
## Source the setup file

To use your new package and executable, first open a new terminal and source your main ROS 2 installation.

Then, from inside the ros2_ws directory, run the following command to source your workspace:

```bash
source install/local_setup.bash
```

To run the executable you created using the --node-name argument during package creation, enter the command:

```bash
ros2 run my_package my_node
```