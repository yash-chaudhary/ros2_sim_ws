# refer to https://www.ros.org/reps/rep-2001.html?fbclid=IwAR3skEz4c_LO0KLoRAw6SPZBmN4hTF00CWArge8Ipr39bwdhCRBJXaRJCQ0#humble-hawksbill-may-2022-may-2027
# refer to https://packages.ros.org/

FROM osrf/ros:humble-desktop-full
SHELL ["/bin/bash", "-c"]

# set container working directory
WORKDIR /ros2_sim_ws

# add additional required packages
RUN apt-get update && apt-get upgrade -y && apt-get install -y \
    curl \
    gnupg \
    lsb-release \
    nano \  
    python3-pip \
    ros-dev-tools \
    ros-humble-joint-state-publisher \
    ros-humble-hardware-interface \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-ros-gz \
    ros-humble-robot-localization \
    ros-humble-robot-state-publisher \
    ros-humble-slam-toolbox \
    ros-humble-xacro \
    software-properties-common \
    && rm -rf /var/lib/apt/lists/*

# install external package topic_tools and rosdeps
COPY ws_deps.yaml /tmp
RUN mkdir src && vcs import --input /tmp/ws_deps.yaml src
RUN apt update && apt-get upgrade -y && rosdep install --from-paths src --ignore-src -r -y 

# copy remaining contents of cloned git repo
COPY . /ros2_sim_ws

# build ros ros packages
RUN source /opt/ros/humble/setup.bash --extend && colcon build --symlink-install --executor sequential

# setup bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
RUN echo "source ./install/setup.bash" >> /root/.bashrc
RUN echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> /root/.bashrc
RUN echo "export GZ_VERSION=fortress"

# run entrypoint script when creating docker container from this image
COPY entrypoint.sh /
ENTRYPOINT [ "/entrypoint.sh" ]
