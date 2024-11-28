#!/usr/bin/env bash

gnome-terminal --title "simulation" -- bash -c "source /home/moon/ws/ws_amr/devel/setup.bash && roslaunch livox_laser_simulation barn_world.launch"