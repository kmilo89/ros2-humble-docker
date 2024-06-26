FROM osrf/ros:humble-desktop-full

# Example of installing programs
RUN apt-get update \
    && apt-get install -y \
    nano \
    vim \
    && rm -rf /var/lib/apt/lists/*

# Create a non-root user
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
  && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
  && mkdir /home/$USERNAME/.config && chown $USER_UID:$USER_GID /home/$USERNAME/.config

# Set up sudo
RUN apt-get update \
  && apt-get install -y sudo \
  && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME\
  && chmod 0440 /etc/sudoers.d/$USERNAME \
  && rm -rf /var/lib/apt/lists/*

RUN apt-get update \
  && apt-get -y install ros-humble-turtlebot3-simulations \
  && apt-get -y install ros-humble-ros-gz \
  && apt-get -y install ros-humble-gazebo-* \
  && apt-get -y install ros-humble-cartographer \
  && apt-get -y install ros-humble-cartographer-ros \
  && apt-get -y install ros-humble-navigation2 \
  && apt-get -y install ros-humble-nav2-bringup \
  && apt-get -y install ros-humble-dynamixel-sdk \
  && apt-get -y install ros-humble-turtlebot3-msgs \
  && apt-get -y install ros-humble-turtlebot3 \
  && rm -rf /var/lib/apt/lists/*

# Copy the entrypoint and bashrc scripts so we have 
# our container's environment set up correctly
COPY entrypoint.sh /entrypoint.sh
COPY bashrc /home/${USERNAME}/.bashrc

RUN mkdir /ros2_ws
WORKDIR /ros2_ws

# Set up entrypoint and default command
ENTRYPOINT ["/bin/bash", "/entrypoint.sh"]
CMD ["bash"]