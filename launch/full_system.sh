#!/bin/bash
export ROSCONSOLE_FORMAT='[${severity}] [${node}]: ${message}'
mkdir `rospack find box_finder`/map
roslaunch box_finder automapping.launch
roslaunch box_finder move_to_table.launch
