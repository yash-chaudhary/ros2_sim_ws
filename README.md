# ROS2 Navigation Simulation

This repository aims to provide a foundation for simulating robots with the Robot Operating System 2 (ROS2).

Using simultaneous localisation and mapping (SLAM), a map of a virtual world is generated. Then using the Navigation 2 stack, a robot is able
to navigate through the virtual world filled with obstacles.

Workspace environment setup:
* ROS2 distribution: [Humble Hawksbill](https://docs.ros.org/en/humble/Installation.html) (humble)
* Simulation: [Gazebo-Fortress](https://gazebosim.org/docs/fortress/getstarted) (Gazebo Sim)
* Visualisation: Rviz2
* Docker Engine / Docker Desktop
* Note that there was a recent breaking change in [gz_ros2_control](https://github.com/ros-controls/gz_ros2_control) so the repo is manually installed for now.

## ROS Packages

Inside `src` folder:

* `sam_bot_description` - Contains robot model including all associated files such as meshes, textures and materials.

* `sam_bot_bringup` - Contains launch files that will launch ROS2 nodes in docker and launch GUI apps in host machine.

* `sam_bot_gazebo` - Contains world files.

* `sam_bot_controller` - Contains ROS2 nodes to control robot.

## Implementation
1. Create URDF of robot and add relevant plugins for control (i.e. ros2_control diff_drive).
2. Define the simulation environment the robot will reside in.
3. Create launch files that start relevant nodes, especially those related to robot TFs.
4. Read user input commands and in conjunction with Nav2 Simple Commander API, control robot navigation.

Below is a graphic depicting the relationship between different parts of this workspace



## Setup
The following steps run through the workspace setup:
1. Install dependencies

    ```bash
    git clone https://github.com/yash-chaudhary/ros2_sim_ws.git
    source /opt/ros/humble/setup.bash
    sudo rosdep init
    rosdep update
    rosdep install --from-paths src
    ```

1. Run setup script

    ```bash
    placeholder
    ```

1. Source the workspace

    ```bash
    placeholder
    ```

1. Launch the simulation

    ```bash
    placeholder
    ```





## Bugs/Limitations/Issues
* \[Limitation\] Not able to run Gazebo-Fortress seamlessly in Docker container
    * Used OSRF Docker image to install full desktop for GUI applications
    * Used x11 to enable X server in container to use host display
    * Able to launch Gazebo-Fortress and RViz2
    * \[Issue\] Gazebo-Fortress could not render model and physics engine, so robot not able to sense surroundings causing issues in RViz2
    * \[Tmp Fix\] Run GUI apps in host machine and use Docker container to run ROS2 nodes
* \[Limitation\] Omission of [X1 Config 6 SDF format robot](https://app.gazebosim.org/OpenRobotics/fuel/models/X1%20Config%206)
    * Able to load SDF model into world
    * Added sensors, robot_state_publisher, joint_state_publisher, odometry plugins
    * Added DiffDrive plugin
    * SDF model loaded in Gazebo-Fortress
    * \[Issue\] Model meshes resource could not be found and loaded by RViz2 (despite using proper env-hooks)
    * \[Tmp Fix\] Use sam_bot URDF from nav2 stack docs to ensure a robot could be visualised in RViz
* \[Issue\] Unable to create [moon world](https://app.gazebosim.org/OpenRobotics/fuel/models/Apollo15%20Landing%20Site%20Heightmap%201000x1000%20meters?fbclid=IwAR1pLdfhnXSIh05fvZ3V84veMrEM5-CD4LSQFrUtQ19ZjxCCOwCKv9LLWaM)  due to unloadable assets 
