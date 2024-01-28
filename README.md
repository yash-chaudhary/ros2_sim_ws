# ROS2 Navigation Simulation

This repository aims to provide a foundation for simulating robots with the Robot Operating System 2 (ROS2). This repository builds on these templates: [repo1](https://github.com/gazebosim/ros_gz_project_template/tree/fortress), [repo2](https://github.com/art-e-fact/navigation2_ignition_gazebo_example).

Using simultaneous localisation and mapping (SLAM), a map of a virtual world is generated. Then using the Navigation 2 stack, a robot is able
to navigate through the virtual world filled with obstacles. 

Workspace environment setup:
* ROS2 distribution: [Humble Hawksbill](https://docs.ros.org/en/humble/Installation.html) (humble)
* Simulation: [Gazebo-Fortress](https://gazebosim.org/docs/fortress/getstarted) (Gazebo Sim)
* Visualisation: Rviz2
* Docker Engine / Docker Desktop

Repository overview:
* Dockerfile: Instructions to produce working docker image
* entrypoint.sh: Docker container entrypoint script
* ws_deps.yaml: Docker image dependancies downloaded from source
* Makefile: Make commands to create/destroy docker environment
* assets: Images/videos for README

## UPDATE 🔄
Success! Docker + ROS2 + Nav2 + Gazebo Sim + RViz2 are completely packaged! 

What was the problem? <br />
<br />
Short answer: gz_ros2_control binary package <br />
<br />
Long answer: <br />
![gz_ros2_control issue](https://github.com/yash-chaudhary/ros2_sim_ws/blob/main/assets/gz_ros2_control_binary_install_error.png)

**The solution: build with gz_ros2_control from source**

## Some VM Pointers
It's probably best to dual boot you're computer with Ubuntu for maximum performance and efficiency. This option was not viable for so I used VMware Workstation 17 (free edition).
1. Download VMware Workstation (version 17 in my case)
2. Download Ubuntu Jammy (22.04) LTS Image (.iso) file 
3. Run through VM setup wizard and **IMPORTANTLY, DISABLE 3D acceleration in display options**  if you don't do this everything breaks
4. NOTE: As we are disabling 3D acceleration, our simulation and visualisation will drop to very low FPS. So it's a better option to dual boot your computer. When a fix comes out such that Gazebo Sim can be rendered with 3D acceleration, the FPS should improve dramatically.

This is what happens when you don't disable 3D acceleration in VMware Workstation:
![render_issue](https://github.com/yash-chaudhary/ros2_sim_ws/blob/main/assets/GUI_apps_with_3D_accel_enabled_VMWare.png)

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

The Nav2 Simple Commander API provides an interface to easily interact with Nav2 ROS2 action servers to easily integrate robot navigation. This API was highly leveraged for navigation in this workspace.

The user is able to command robot navigation by typing characters on their keyboard. The workspace comes pre-packaged with a list of poses (navigation goals) that are bundled together in a list of waypoints.
The users is able to select between 2 pre-defined waypoint sets and upon selection, the robot will start going to each of the poses defined in the waypoint set. The user is also about to initiate an emergency 
stop by pressing the 's' key. This will stop the robot. Therefore a new node was created that read user input and created new goals for Nav2.

![docker_container_workspace](https://github.com/yash-chaudhary/ros2_sim_ws/blob/main/assets/simulation_visualsation_docker_instance.png)

## Demonstration

Here's a video demo I made so that everyone can follow along: https://vimeo.com/906874396?share=copy#t=0

## Usage
The following steps run through the workspace setup:
1. Clone repository

    ```bash
    git clone https://github.com/yash-chaudhary/ros2_sim_ws.git
    ```
1. Make entrypoint.sh executable (if not already)

    ```bash
    chmod +x entrypoint.sh
    ```

1. Run Makefile help
    ```bash
    # check if you have make installed
    /usr/bin/make --version
    
    # if not then install
    sudo apt install -y make

    # if this doesn't work use this command
    sudo apt install build-essential
    ```

    ```bash
    make help
    ```
    Available targets:
      - build: Creates docker image
      - run_display: Runs docker container and launches simulations and visualisations
      - run_nav: Creates bash process in container to control robot navigation
      - cleanup: Removes image and container
      - help: Show this help message

1. Build image

    ```bash
    # NOTE: depending on sudo permission you may need to run: sudo make build (which runs docker as a sudo user)
    make build
    ```

1. Launch the simulation and visualisation

    ```bash
    make run_display
    ```
    
 1. Launch navigation controller (open a new terminal and run this)

    ```bash
    make run_nav
    ```
    Available commands:
    - Press key 'q' to run navigation routine, waypoint set 1
    - Press key 'w' to run navigation routine, waypoint set 2
    - Press key 's' to execute emergency stop

 1. When done remove image and container

    ```bash
    make cleanup
    ```

## Bugs/Limitations/Issues
* \[Limitation\] Simulation is quite slow in VM
* \[Limitation\] Omission of [X1 Config 6 SDF format robot](https://app.gazebosim.org/OpenRobotics/fuel/models/X1%20Config%206)
    * Able to load SDF model into world
    * Added sensors, robot_state_publisher, joint_state_publisher, odometry plugins
    * Added DiffDrive plugin
    * SDF model loaded in Gazebo-Fortress
    * \[Issue\] Model meshes resource could not be found and loaded by RViz2 (despite using proper env-hooks)
    * \[Tmp Fix\] Use sam_bot URDF from nav2 stack docs to ensure a robot could be visualised in RViz
* \[Limitation\] Need to add unit and integration tests for custom node and launch process
* \[Issue\] Unable to create [moon world](https://app.gazebosim.org/OpenRobotics/fuel/models/Apollo15%20Landing%20Site%20Heightmap%201000x1000%20meters?fbclid=IwAR1pLdfhnXSIh05fvZ3V84veMrEM5-CD4LSQFrUtQ19ZjxCCOwCKv9LLWaM)  due to unloadable assets 
