#!/usr/bin/env bash

IP=`ifconfig tap0 | grep 'inet addr:'  | cut -d: -f2 | awk '{ print $1}'`

echo "my ip: ${IP}"
source $ROSE_SCRIPTS/setup_ROS.sh "/opt/ros/hydro/" "${IP}" "http://$(gethostip -d rosepc1):11311"

export ROBOT_NAME=cockpit

roslaunch rose_gui_application ui.launch

status=$?
if [ $status -ne 0 ]; then
    echo "Press enter to exit"
    read -n 1
fi
