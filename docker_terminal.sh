#!/bin/bash

# Configuration
SERVICE_NAME="ros2_humble_student"
CONTAINER_NAME="ros_humble_container"

# Create necessary folders on the HOST first so Docker doesn't create them as root
# ROS 2 uses 'install' and 'log' instead of 'devel' and 'logs'
for dir in src build install log; do
  if [ ! -d "$dir" ]; then
    mkdir -p "$dir"
  fi
done

# Ensure the current user owns these folders
#sudo chown -R $USER:$USER .

chmod +x build_ws.sh

# Ensure .bash_history exists as a file so Docker doesn't create it as a directory
touch .bash_history

# Allow Docker to communicate with X11 for GUI apps
xhost +local:docker > /dev/null

# Check if the container is already running
if [ -z "$(docker ps -q -f name=$CONTAINER_NAME)" ]; then
    echo "Starting $SERVICE_NAME..."
    docker compose up -d
    sleep 1
fi

echo "Entering ROS 2 Humble Environment..."
# Execute bash inside the container as the 'student' user
docker exec -it --user student $CONTAINER_NAME /bin/bash
