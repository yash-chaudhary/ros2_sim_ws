# ROS2 Navigation Simulation

## Introduction
This repository aims to provide a foundation for simulating robots with the Robot Operating System 2 (ROS2).

Using simultaneous localisation and mapping (SLAM), a map of a virtual world is generated. Then using the Navigation 2 stack, a robot is able
to navigate through the virtual world filled with obstacles.

Workspace environment setup:
* ROS2 distribution: Humble Hawksbill (humble)
* Simulation: Gazebo-Fortress (GazeboSim)
* Visualisation: Rviz2
* Docker Engine / Docker Desktop

## Implementation
Below is a graphic depicting the relationship between different parts of this workspace


## Setup
The following steps run through the workspace setup. 


## Bugs/Limitations/Issues
* \[Limitation\] Not able to Gazebo-Fortress seamlessly in Docker container
    * Used OSRF Docker image to install full desktop for GUI applications
    * Used x11 to enable X server in container to use host display
    * Able to launch Gazebo-Fortress and RViz2
    * \[Issue\] Gazebo-Fortress could not render the physics engine and so robot not able to visualise surroundings causing issues in RViz2
    * \[Tmp Fix\] Run GUI apps in host machine and use Docker container to run ROS2 nodes
* \[Limitation\] Omission of [X1 Config 6 SDF format robot](https://app.gazebosim.org/OpenRobotics/fuel/models/X1%20Config%206)
    * Able to load SDF model into world
    * Added sensors, robot_state_publisher, joint_state_publisher, odometry plugins
    * Added DiffDrive plugin
    * SDF model loaded in Gazebo-Fortress
    * \[Issue\] Model meshes resource could not be found and loaded by RViz2 (despite using proper env-hooks)
    * \[Tmp Fix\] Use sam_bot URDF from nav2 stack docs to ensure a robot could be visualised in RViz
* \[Issue\] Unable to create [moon world](https://app.gazebosim.org/OpenRobotics/fuel/models/Apollo15%20Landing%20Site%20Heightmap%201000x1000%20meters?fbclid=IwAR1pLdfhnXSIh05fvZ3V84veMrEM5-CD4LSQFrUtQ19ZjxCCOwCKv9LLWaM)  due to unloadable assets 


Simulation and visualisation were high priority for this workspace. Initially the goal was run all GUI programs (GazeboSim and RViz) in a docker container along with all ROS2 nodes.
However, after many unsucessful attemps, Gazebo-Fortress was not able to render the map of the virtual world which in turn caused RViz to complain thus rendering the 
simulation environment pretty pointless. Instead a new strategy was taken to run all GUI apps on the host machine and run all ROS2 nodes in a docker container.
