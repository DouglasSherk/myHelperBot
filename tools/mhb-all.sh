#!/bin/sh
echo "Launching core..."
mhb-core.sh &
sleep 5
echo "Launching Arduino and Kinect..."
mhb-serial.sh &
mhb-kinect.sh &
sleep 5
echo "Launching logic..."
mhb-logic.sh &
