#!/usr/bin/env bash

# TODO: (Daehan) change command line
gnome-terminal --title "fast_lio" -- bash -c "source /home/psh/workspace/ros_workspace/lio_ws/devel/setup.bash && roslaunch fast_lio mapping_avia.launch"