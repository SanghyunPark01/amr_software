#!/usr/bin/env bash

# TODO: (Daehan) change command line
gnome-terminal --title "fast_lio" -- bash -c "source /home/moon/ws/ws_amr/devel/setup.bash && roslaunch fast_lio_localization localization_mid360.launch"