#!/bin/bash
#This brings up files necessary for running the Svenzva Revel Pick and Place Exhibit
#Note that roslaunch's stochastic nature necessitated a script that allows deterministic sequencing 
roslaunch svenzva_drivers svenzva_bringup.launch mode:=velocity moveit_for_cart:=true &
sleep 10
roslaunch svenzva_joy svenzva_6axis_custom.launch &
roslaunch svenzva_moveit start_moveit.launch &
sleep 10
rosrun svenzva_utils publish_moveit_collision.py &
