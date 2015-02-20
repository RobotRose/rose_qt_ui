#!/usr/bin/env bash

TOPIC="/cameraOperator/image"
export DISPLAY=:0 #; rosrun image_view image_view image:=${TOPIC} _image_transport:=theora&
rosrun image_view image_view image:=${TOPIC} _image_transport:=theora&
PROCESS=$!

echo "waiting 1s to make image_view (pid $PROCESS) maximized"
sleep 1
wmctrl -r "${TOPIC}" -b add,maximized_vert,maximized_horz #Topic is also name of the window
echo "waiting 10s to certainly make image_view maximized"
sleep 10 #1 second may be too fast, so try again after 10s
wmctrl -r "${TOPIC}" -b add,maximized_vert,maximized_horz #Topic is also name of the window

echo "foregrounding image_view process"
wait "${PROCESS}"

