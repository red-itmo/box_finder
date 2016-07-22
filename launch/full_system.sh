#!/bin/bash
export ROSCONSOLE_FORMAT='[${severity}] [${node}]: ${message}'
gnome-terminal -e "bash -c \"roscore ; exec bash\"" &
sleep 2
gnome-terminal -e "bash -c \"rosparam set use_sim_time true ; rosrun stage_ros stageros `rospack find box_finder`/map/debug.world ; exec bash\"" &
sleep 2
roslaunch box_finder automapping.launch
roslaunch box_finder move_to_table.launch