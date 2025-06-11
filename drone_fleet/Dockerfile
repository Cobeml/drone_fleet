FROM osrf/ros:humble-desktop

# Install additional ROS2 packages
RUN apt-get update && apt-get install -y \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-pcl-ros \
    ros-humble-rosbridge-suite \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-plugins \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-xacro \
    python3-pip \
    python3-colcon-common-extensions \
    git \
    wget \
    && rm -rf /var/lib/apt/lists/*

# Install PX4-related packages
RUN pip3 install px4tools pymavlink

# Create workspace
WORKDIR /workspace

# Set up environment
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /workspace/install/setup.bash" >> ~/.bashrc

# Expose ports for ROSbridge
EXPOSE 9090

CMD ["bash"]