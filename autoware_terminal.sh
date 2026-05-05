#!/bin/bash

# --- Configuration ---
IMAGE_NAME="mohsen_aw:full"
CONTAINER_NAME="autoware_dev"
CONTAINER_WORKDIR="/ros2_ws"

# Host paths to mount
HOST_ROS_WS="/home/$USER/ros2_ws_"
HOST_MAP_DIR="/home/$USER/autoware_map"

FILE_TO_SOURCE="setup.bash"

# Create necessary folders on the HOST first so Docker doesn't create them as root
# ROS 2 uses 'install' and 'log' instead of 'devel' and 'logs'
for dir in src build install log; do
  if [ ! -d "$dir" ]; then
    mkdir -p "$dir"
  fi
done

# --- Logic: Check if container is running ---
# We use -q (quiet/ID only) and filter by name. 
# If the output is NOT empty, the container exists.
RUNNING=$(docker ps -q -f "name=^/${CONTAINER_NAME}$")

if [ -n "$RUNNING" ]; then
    echo "Container '$CONTAINER_NAME' is already running. Opening new terminal..."
    docker exec -it "$CONTAINER_NAME" bash -c "source $FILE_TO_SOURCE && bash"
else
    # Check if a stopped container with that name exists and remove it to avoid conflicts
    if [ "$(docker ps -aq -f "name=^/${CONTAINER_NAME}$")" ]; then
        echo "Removing old stopped container..."
        docker rm "$CONTAINER_NAME" > /dev/null
    fi

    echo "Starting a new container instance: '$CONTAINER_NAME'..."
    
    xhost +local:docker > /dev/null

    docker run -it --rm \
        --name "$CONTAINER_NAME" \
        --privileged \
        --net=host \
        --env="DISPLAY=$DISPLAY" \
        --env="QT_X11_NO_MITSHM=1" \
        -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
        -v "$HOST_ROS_WS":"$CONTAINER_WORKDIR" \
        -v "$HOST_MAP_DIR":/autoware_map \
        --workdir "$CONTAINER_WORKDIR" \
        "$IMAGE_NAME" \
        bash -c "source $FILE_TO_SOURCE && bash"
fi