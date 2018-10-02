#!/bin/sh

XSOCK=/tmp/.X11-unix
XAUTH=/home/$USER/.Xauthority
SHARED_DIR=/home/autoware/shared_dir
HOST_DIR=/home/$USER/shared_dir

if [ "$1" = "kinetic" ] || [ "$1" = "indigo" ]
then
    echo "Use $1"
else
    echo "Select distribution, kinetic|indigo"
    exit
fi

if [ "$2" = "" ]
then
    # Create Shared Folder
    mkdir -p $HOST_DIR
else
    HOST_DIR=$2
fi
echo "Shared directory: ${HOST_DIR}"

nvidia-docker run \
    -it --rm \
    --volume=$XSOCK:$XSOCK:rw \
    --volume=$XAUTH:$XAUTH:rw \
    --volume=$HOST_DIR:$SHARED_DIR:rw \
    -v /usr/lib/x86_64-linux-gnu/libGL.so.1.7.0:/usr/lib/x86_64-linux-gnu/libGL.so.1.7.0 \
    -v /usr/lib/x86_64-linux-gnu/libGL.so.1.7:/usr/lib/x86_64-linux-gnu/libGL.so.1.7 \
    -v /usr/lib/x86_64-linux-gnu/libGL.so.1:/usr/lib/x86_64-linux-gnu/libGL.so.1 \
    -v /usr/lib/x86_64-linux-gnu/libGLX.so.0:/usr/lib/x86_64-linux-gnu/libGLX.so.0 \
    -v /usr/lib/x86_64-linux-gnu/libGLdispatch.so.0:/usr/lib/x86_64-linux-gnu/libGLdispatch.so.0 \
    --env="XAUTHORITY=${XAUTH}" \
    --env="DISPLAY=${DISPLAY}" \
    -u autoware \
    --privileged -v /dev/bus/usb:/dev/bus/usb \
    --net=host \
    autoware-$1
