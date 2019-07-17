#!/bin/sh
echo "launch darknet"
gnome-terminal -x bash -c "rosnode kill /darknet_ros"

sleep 6

echo "objection detection"
gnome-terminal -x bash -c "rosnode kill /object_detection"

sleep 3

echo "move base adjust"
gnome-terminal -x bash -c "rosnode kill /odom_adjust"

sleep 3

echo "grasp attempt"
gnome-terminal -x bash -c "rosnode kill /grasp_target_object"

sleep 3


exit 0 
