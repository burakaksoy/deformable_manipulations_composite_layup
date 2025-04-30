#!/usr/bin/env bash

export ROS_WS=/home/burak/catkin_ws_deformable
export ROS_VERSION=/opt/ros/noetic
source $ROS_VERSION/setup.bash
source $ROS_WS/devel/setup.bash
export PATH=$ROS_ROOT/bin:$PATH
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$ROS_WS
export ROS_MASTER_URI=http://192.168.1.100:11311/
export ROS_IP=192.168.1.200

# Manually set DISPLAY for remote launch
# export DISPLAY=:1 # For Kinect remote launch, THIS COULD BE :0 AS WELL, CHECK WITH "echo $DISPLAY"

# Auto-detect active DISPLAY
export DISPLAY=$(who | grep '(:' | awk '{print $5}' | tr -d '()')
# fallback to default if above fails
if [ -z "$DISPLAY" ]; then
    export DISPLAY=:0
fi

exec "$@"