FROM ubuntu:20.04

# Avoid user interaction with tzdata
ENV DEBIAN_FRONTEND=noninteractive

# Replace 'rob' with your actual username (unless you are called rob)
# Optionally, set the UID and GID to match your host user's UID and GID
ARG USERNAME=rob
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Update and install necessary packages
RUN apt-get update && apt-get install -y \
    curl \
    gnupg2 \
    lsb-release \
    nano \
    sudo \
    usbutils \
    python3-pip \
    python3-dev \
    && rm -rf /var/lib/apt/lists/*

# Install Raspberry Pi GPIO libraries for Python3
RUN pip3 install RPi.GPIO gpiozero smbus2 rpi_ws281x pyserial

# Add the ROS 2 and Gazebo repositories
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - \
    && echo "deb http://packages.ros.org/ros2/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list \
    && curl -s http://packages.osrfoundation.org/gazebo.key | apt-key add - \
    && sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'

# Install ROS 2 Desktop, Gazebo, and GUI tools
RUN apt-get update && apt-get install -y \
    ros-foxy-desktop \
    ros-foxy-xacro \
    ros-foxy-joint-state-publisher-gui \
    gazebo11 \
    ros-foxy-gazebo-ros-pkgs \
    x11-apps \
    python3-colcon-common-extensions \
    build-essential \
    cmake \
    && rm -rf /var/lib/apt/lists/*

# Create a user with the same user ID as the host user
# and add user to 'sudo' group
# Create a group with the specified GID and a user with the specified UID and GID.
# Then, set up passwordless sudo access for the user.
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID --create-home --shell /bin/bash $USERNAME \
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    # Adding the user to the dialout group
    && usermod -a -G dialout $USERNAME

# Set the environment variable for the display
ENV DISPLAY=host.docker.internal:0

# Switch to the user
USER $USERNAME

# Set the working directory to the user's home directory
WORKDIR /home/$USERNAME

# Source the ROS 2 setup file in the user's bashrc
RUN echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc

# Setup entrypoint
ENTRYPOINT ["/bin/bash"]
