FROM ubuntu:20.04

# Avoid user interaction with tzdata
ENV DEBIAN_FRONTEND=noninteractive

# Update and install necessary packages
RUN apt-get update && apt-get install -y \
    curl \
    gnupg2 \
    lsb-release \
    && rm -rf /var/lib/apt/lists/*

# Add the ROS 2 and Gazebo repositories
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - \
    && echo "deb http://packages.ros.org/ros2/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list \
    && curl -s http://packages.osrfoundation.org/gazebo.key | apt-key add - \
    && sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'

# Install ROS 2 Desktop and Gazebo
RUN apt-get update && apt-get install -y \
    ros-foxy-desktop \
    gazebo11 \
    ros-foxy-gazebo-ros-pkgs \
    && rm -rf /var/lib/apt/lists/*

# Install tools to enable GUI applications (e.g., RViz, rqt) to run
RUN apt-get update && apt-get install -y \
    x11-apps \
    && rm -rf /var/lib/apt/lists/*

# Setup environment to enable GUI applications to display on the host
ENV DISPLAY=host.docker.internal:0

# Source the ROS 2 setup file
RUN echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc

# Setup entrypoint
ENTRYPOINT ["/bin/bash"]
