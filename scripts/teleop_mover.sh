#!/bin/bash
source ~/ros/kinetic/devel/setup.bash 

gnome-terminal --zoom=0.5 --geometry=+0+0 --tab -e 'bash -c "roslaunch task_editor robot_bringup.launch "'
