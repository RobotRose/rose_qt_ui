#!/usr/bin/env bash

echo "Starting Twinkle"
twinkle --hide --call sip:rose@10.8.0.1 --immediate
PROCESS=$!
echo "Twinkle is running with pid $PROCESS"

echo "waiting 1s to make Twinkle (pid $PROCESS) hidden"
sleep 1
wmctrl -r "Twinkle" -b add,below,hidden #Topic is also name of the window
echo "waiting 10s to certainly make Twinkle hidden"
sleep 10 #1 second may be too fast, so try again after 10s
wmctrl -r "Twinkle" -b add,below,hidden #Topic is also name of the window

echo "foregrounding Twinkle process"
wait "${PROCESS}"
