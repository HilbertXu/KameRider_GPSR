#!/bin/sh
echo "launch darknet"
gnome-terminal -x bash -c "roslaunch darknet_ros darknet_ros.launch"

sleep 6

echo "objection detection"
gnome-terminal -x bash -c "rosrun kamerider_image_core object_detection"

sleep 3

echo "move base adjust"
gnome-terminal -x bash -c "rosrun kamerider_navigation_core odom_adjust_for_grasp.py"

sleep 3

echo "grasp attempt"
gnome-terminal -x bash -c "rosrun turtlebot_arm_moveit_demos grasp_target_object.py"

sleep 3


exit 0 
