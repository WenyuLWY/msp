#!/bin/bash


source config.env

if docker container inspect "$CONTAINER_NAME" > /dev/null 2>&1; then
      echo "Container already exists."
      echo "Stopping and removing the existing container..."
      docker stop $CONTAINER_NAME
      docker rm $CONTAINER_NAME
fi


PKG_ROOT_DIR=$(dirname "$(dirname "$(readlink -f "$0")")")
MOUNT_ARGS=""
# MOUNT_ARGS="$MOUNT_ARGS -v $PKG_ROOT_DIR/.vscode:/root/catkin_ws/src/.vscode"
# MOUNT_ARGS="$MOUNT_ARGS -v $PKG_ROOT_DIR/.git:/root/catkin_ws/src/.git"


mkdir -p $PKG_ROOT_DIR/tmp/{devel,build,logs}
for path in "$PKG_ROOT_DIR"/*; do
    name=$(basename "$path")
    if [ "$name" != "tmp" ]; then
        MOUNT_ARGS="$MOUNT_ARGS -v $path:/home/user/catkin_ws/src/$name"
    fi
done
MOUNT_ARGS="$MOUNT_ARGS -v $PKG_ROOT_DIR/tmp/build:/home/user/catkin_ws/build"
MOUNT_ARGS="$MOUNT_ARGS -v $PKG_ROOT_DIR/tmp/devel:/home/user/catkin_ws/devel"
MOUNT_ARGS="$MOUNT_ARGS -v $PKG_ROOT_DIR/tmp/logs:/home/user/catkin_ws/logs"

# -e XDG_RUNTIME_DIR=/tmp/runtime-root \
# -v ~/gazebo_models:/home/user/.gazebo/models \
if grep -qi microsoft /proc/version; then # Check if running in WSL
  docker run -it --name $CONTAINER_NAME \
      --gpus all --device=/dev/dxg \
      -e DISPLAY=$DISPLAY \
      -e WAYLAND_DISPLAY=$WAYLAND_DISPLAY \
      -e PULSE_SERVER=$PULSE_SERVER \
      -e XDG_RUNTIME_DIR=/tmp/runtime-root \
      -e LD_LIBRARY_PATH=/usr/lib/wsl/lib \
      -v /mnt/wslg:/mnt/wslg \
      -v /tmp/.X11-unix:/tmp/.X11-unix \
      -v /usr/lib/wsl:/usr/lib/wsl \
      $MOUNT_ARGS \
      $IMAGE_NAME bash
else # Running in ubuntu environment
  XAUTH="$(dirname "$(readlink -f "$0")")/.docker.$CONTAINER_NAME.xauth"

  rm -f "$XAUTH"
  touch "$XAUTH"
  xauth nlist "$DISPLAY" | sed -e 's/^..../ffff/' | xauth -f "$XAUTH" nmerge -
  chmod 644 "$XAUTH"

  docker run -it --name $CONTAINER_NAME \
      --user user \
      --gpus all -e NVIDIA_VISIBLE_DEVICES=all -e NVIDIA_DRIVER_CAPABILITIES=all \
      --privileged --net=host -v /dev:/dev \
      -e XDG_RUNTIME_DIR=/home/user/tmp/runtime-root \
      -e DISPLAY=$DISPLAY -e TERM -e QT_X11_NO_MITSHM=1 \
      -e XAUTHORITY=/home/user/.docker.$CONTAINER_NAME.xauth \
      -v $XAUTH:/home/user/.docker.$CONTAINER_NAME.xauth \
      -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
      -v /etc/localtime:/etc/localtime:ro \
      -v ~/.gazebo/models:/home/user/.gazebo/models \
      $MOUNT_ARGS \
      $IMAGE_NAME bash
fi

