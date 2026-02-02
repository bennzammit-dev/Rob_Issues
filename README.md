# Rob_Issues
Code pertaining mainly to environment generation in ros2 gz sim, has issues with spawning dynamic elements.

Summary by chat-gpt:

This repository contains a ROS 2 Jazzy workspace for a Gazebo Sim–based robotics assignment involving a robot called Sortabot and several supporting nodes. The purpose of the project is to run a simulated environment in Gazebo, spawn a robot, spawn task-related objects such as bins and items, and then implement control and decision-making logic on top of that environment.

The project is structured as a ROS 2 workspace. The main workspace directory is called sortabot_ws. Due to GitHub limitations, the workspace itself could not be uploaded exactly as it exists locally. Inside this workspace, the important directory is src/sortabot, which is the actual ROS 2 package. The sortabot folder visible in this repository corresponds to that package.

Within the sortabot package, there are several key folders and files.

The launch folder contains the launch file sortabot.launch.py. This file is responsible for starting Gazebo Sim, loading the simulation world, and launching all required ROS 2 nodes, including the world manager, the robot controller, and the childabot nodes.

The worlds folder contains the Gazebo world definition file, typically named sortabot_base.world. This file defines the static simulation environment, including the ground plane, lighting, and other global world properties. At the moment, this is the only component that reliably loads and appears in the simulation.

The sortabot Python package folder contains the main ROS 2 nodes implemented in Python. The world_manager.py file is responsible for dynamically spawning entities into the Gazebo world at runtime. These entities include bins, objects, and other task-related elements required for the assignment. The world manager also publishes a task definition on a ROS topic that the robot controller is expected to consume.

The robot_controller.py file contains the logic for controlling the main Sortabot robot. It is intended to subscribe to task information and issue movement or action commands based on that information. The childabot.py file represents supporting robots or agents that exist in the environment and interact with the main robot or the task.

The resource folder is intended to contain robot models, meshes, and other assets used by Gazebo and ROS. Some files may appear missing or incomplete in the repository due to GitHub restrictions on certain file types or empty directories, but they are present in the local workspace.

The package.xml, setup.py, and setup.cfg files define the ROS 2 package metadata and Python packaging configuration. These files allow the package to be built and installed using colcon as part of the ROS 2 workspace.

From my perspective, the main issue with the project is that only the Gazebo world file spawns correctly. When the launch file is executed, Gazebo Sim opens as expected and loads the world file, but none of the dynamic entities appear in the simulation. This includes the robot, bins, objects, and any other entities that are meant to be spawned at runtime.

The intended behaviour is for the world_manager node to connect to Gazebo Sim spawn services and dynamically create entities using SDF XML definitions. In practice, this does not work. The node either fails to find a compatible spawn service or attempts to use service names or interfaces that are not available or compatible with the version of Gazebo Sim used in ROS 2 Jazzy.

As a result, the world_manager node exits with errors indicating that no usable spawn service could be found, even though Gazebo is running and exposing services. Because of this failure, the simulation never progresses beyond the static world defined in the .world file.

This issue appears to be related to differences between Gazebo Classic and Gazebo Sim, particularly in how entity spawning is handled and how services are exposed to ROS 2. While the assignment and lecture material use Gazebo Sim, many existing examples and older code rely on Gazebo Classic, which leads to confusion and incompatibilities when trying to adapt code for Jazzy.

In summary, the workspace builds and launches successfully, Gazebo Sim runs correctly, and the static world loads as expected. However, the dynamic spawning of robots and task elements does not function, leaving the simulation incomplete and preventing further progress on the assignment logic.

Running instructions:

Ensure Gazebo Sim (gz-sim) and ROS–Gazebo bridges are installed

You can verify Gazebo Sim is installed by running:

gz sim --version

You can install by running:

sudo apt update
sudo apt install ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-interfaces

Create a ROS 2 workspace:

mkdir -p ~/sortabot_ws/src
cd ~/sortabot_ws/src

Put repo in workspace such that you have sortabot_ws/src/sortabot

Build the workspace:

cd ~/sortabot_ws
colcon build

Source the workspace:

Every new terminal that is used to run the project must source the workspace:

source ~/sortabot_ws/install/setup.bash

Run the simulation:

Launch the simulation using the provided launch file:

ros2 launch sortabot sortabot.launch.py

This will start:

Gazebo Sim with the base world file

The world_manager node

The robot_controller node

The childabot node(s)

At this point, Gazebo should open and load the static world defined in the .world file.

The world file (sortabot_base.world) should load correctly and be visible in Gazebo.

However the dynamically spawned entities (bins, objects, obstacles and bots) do not appear.
