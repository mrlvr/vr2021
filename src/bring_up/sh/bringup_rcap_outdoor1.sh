#!/bin/bash

world_launch="rcap_outdoor1.launch"
spawn_launch="spawn_rcap_outdoor1.launch"
vrpath="/home/mrlvr/Robocup2018RVRL"

tab1="bash -c '. setup.bash';'roslaunch setup $world_launch';bash"
tab2="bash -c '. setup.bash';'sleep 5';'roslaunch robot_description $spawn_launch';bash"
gnome-terminal --tab --working-directory="$vrpath" -e "$tab1" --tab --working-directory="$vrpath" -e "$tab2"
exit 0



