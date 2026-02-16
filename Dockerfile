# Base image: ROS 2 Humble (or your desired ROS 2 distribution)
FROM osrf/ros:humble-desktop

# Install TurtleBot 3 packages and tools
RUN apt-get update && apt-get upgrade -y && apt-get install -y \
    sudo \
    ros-humble-gazebo-* \
    ros-humble-cartographer \
    ros-humble-cartographer-ros \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-turtlebot3 \
    ros-humble-turtlebot3-msgs \
    ros-humble-tf-transformations \
    python3-colcon-common-extensions \
    build-essential \
    git \
    gedit \
    python3-pip && \
    pip3 install setuptools==58.2.0 && \
    pip3 install transforms3d && \
    apt-get clean && rm -rf /var/lib/apt/lists/*
    


# Set environment variables
# ENV TURTLEBOT3_MODEL=burger

# Add a root user for student
RUN useradd -ms /bin/bash student && echo "student ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers
USER student
WORKDIR /home/student


# Source ROS 2 setup scripts in the bash environment
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo 'export ROS_DOMAIN_ID=30 #TURTLEBOT3' >> ~/.bashrc
RUN echo 'export TURTLEBOT3_MODEL=burger' >> ~/.bashrc
RUN echo 'source ~/ws/install/setup.bash' >> ~/.bashrc
RUN echo 'source /usr/share/gazebo/setup.sh' >> ~/.bashrc
RUN echo "export ROS_LOCALHOST_ONLY=1" >> ~/.bashrc

# Default command
CMD ["bash"]

