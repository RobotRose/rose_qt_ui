#!/usr/bin/env bash

echo "Starting Twinkle"
export DISPLAY=:0
twinkle --hide  & #-c
PROCESS=$!
echo "Twinkle is running with pid $PROCESS"

echo "waiting 1s to make Twinkle (pid $PROCESS) maximized"
sleep 1
wmctrl -r "Twinkle" -b add,below #Topic is also name of the window
echo "waiting 10s to certainly make Twinkle maximized"
sleep 10 #1 second may be too fast, so try again after 10s
wmctrl -r "Twinkle" -b add,below #Topic is also name of the window

echo "foregrounding Twinkle process"
wait "${PROCESS}"
