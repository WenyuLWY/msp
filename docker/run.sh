#!/bin/bash


source config.env

if [ "$(docker ps -a -q -f name=$CONTAINER_NAME)" ]; then
  docker stop $CONTAINER_NAME
  docker rm $CONTAINER_NAME
fi



PKG_ROOT_DIR=$(dirname "$(dirname "$(readlink -f "$0")")")
MOUNT_ARGS=""
MOUNT_ARGS="$MOUNT_ARGS -v $PKG_ROOT_DIR/.vscode:/root/catkin_ws/src/.vscode"
MOUNT_ARGS="$MOUNT_ARGS -v $PKG_ROOT_DIR/.git:/root/catkin_ws/src/.git"


mkdir -p $PKG_ROOT_DIR/tmp/{devel,build,logs}
for path in "$PKG_ROOT_DIR"/*; do
    name=$(basename "$path")
    if [ "$name" != "tmp" ]; then
        MOUNT_ARGS="$MOUNT_ARGS -v $path:/root/catkin_ws/src/$name"
    fi
done
MOUNT_ARGS="$MOUNT_ARGS -v $PKG_ROOT_DIR/tmp/build:/root/catkin_ws/build"
MOUNT_ARGS="$MOUNT_ARGS -v $PKG_ROOT_DIR/tmp/devel:/root/catkin_ws/devel"
MOUNT_ARGS="$MOUNT_ARGS -v $PKG_ROOT_DIR/tmp/logs:/root/catkin_ws/logs"

# -e XDG_RUNTIME_DIR=/tmp/runtime-root \
if grep -qi microsoft /proc/version; then # Check if running in WSL
  docker run -it --name $CONTAINER_NAME \
      --gpus all --device=/dev/dxg \
      -e DISPLAY=$DISPLAY \
      -e WAYLAND_DISPLAY=$WAYLAND_DISPLAY \
      -e PULSE_SERVER=$PULSE_SERVER \
      -e XDG_RUNTIME_DIR=/mnt/wslg/runtime-dir \
      -e LD_LIBRARY_PATH=/usr/lib/wsl/lib \
      -v /mnt/wslg:/mnt/wslg \
      -v /tmp/.X11-unix:/tmp/.X11-unix \
      -v /usr/lib/wsl:/usr/lib/wsl \
      -v ~/gazebo_models:/root/.gazebo/models \
      $MOUNT_ARGS \
      $IMAGE_NAME bash
else # Running in ubuntu environment
  XAUTH="$(dirname "$(readlink -f "$0")")/.docker.$CONTAINER_NAME.xauth"

  rm -f "$XAUTH"
  touch "$XAUTH"
  xauth nlist "$DISPLAY" | sed -e 's/^..../ffff/' | xauth -f "$XAUTH" nmerge -
  chmod 644 "$XAUTH"

  docker run -it --name $CONTAINER_NAME \
      --gpus all -e NVIDIA_VISIBLE_DEVICES=all -e NVIDIA_DRIVER_CAPABILITIES=all \
      --privileged --net=host -v /dev:/dev \
      -e DISPLAY=$DISPLAY -e TERM -e QT_X11_NO_MITSHM=1 \
      -e XAUTHORITY=/root/.docker.$CONTAINER_NAME.xauth \
      -v $XAUTH:/root/.docker.$CONTAINER_NAME.xauth \
      -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
      -v /etc/localtime:/etc/localtime:ro \
      -v ~/.gazebo/models:/root/.gazebo/models \
      $MOUNT_ARGS \
      $IMAGE_NAME bash
fi

