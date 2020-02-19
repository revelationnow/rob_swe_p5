#!/usr/bin/env bash
xterm -e "roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=`pwd`/src/my_robot/worlds/myworld.world" &
sleep 5

xterm -e "roslaunch turtlebot_gazebo gmapping_demo.launch custom_gmapping_launch_file:=`pwd`/src/turtlebot_apps/turtlebot_navigation/launch/includes/gmapping/gmapping.launch.xml" &
sleep 5

xterm -e "roslaunch turtlebot_teleop keyboard_teleop.launch" &
sleep 5

xterm -e "roslaunch turtlebot_rviz_launchers view_navigation.launch"
