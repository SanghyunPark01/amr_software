#!/usr/bin/env bash

# TODO: (youngtae) change command line
gnome-terminal --title "navigation" -- bash -c "source /home/moon/ws/ws_amr/devel/setup.bash && roslaunch nav navigation.launch 2> >(grep -v TF_REPEATED_DATA buffer_core)"