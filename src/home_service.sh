#!/usr/bin/env bash
xterm -e "roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=`pwd`/src/my_robot/worlds/myworld.world" &
sleep 5

xterm -e "roslaunch turtlebot_gazebo amcl_demo.launch map_file:=`pwd`/src/my_robot/maps/test_map.yaml" &
sleep 5

xterm -e "roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 5

xterm -e "rosrun  add_markers add_markers" &
sleep 5

xterm -e "rosrun pick_objects pick_objects" &
