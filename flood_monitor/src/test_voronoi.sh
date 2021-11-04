#!/bin/sh
rosservice call /drone01/enable_motors true
rosservice call /drone02/enable_motors true
rosservice call /drone03/enable_motors true
rosservice call /drone04/enable_motors true
rosservice call /drone05/enable_motors true
rosservice call /drone06/enable_motors true
rosservice call /drone07/enable_motors true
rosservice call /drone08/enable_motors true
rosservice call /drone09/enable_motors true
rosservice call /drone10/enable_motors true
rosservice call /drone11/enable_motors true
rosservice call /drone12/enable_motors true
rosservice call /drone13/enable_motors true
rosservice call /drone14/enable_motors true
rosservice call /drone15/enable_motors true
rosservice call /drone16/enable_motors true
rosservice call /drone17/enable_motors true
sleep 1


gnome-terminal --tab -- rosrun flood_monitor initialize_drone.py

sleep 5

gnome-terminal --tab -- rosrun flood_monitor drone12.py  
gnome-terminal --tab -- rosrun flood_monitor drone14.py
gnome-terminal --tab -- rosrun flood_monitor drone11.py

sleep 5

gnome-terminal --tab -- rosrun flood_monitor drone16.py
gnome-terminal --tab -- rosrun flood_monitor drone03.py
gnome-terminal --tab -- rosrun flood_monitor drone17.py 

sleep 5

gnome-terminal --tab -- rosrun flood_monitor drone15.py 
gnome-terminal --tab -- rosrun flood_monitor drone13.py
gnome-terminal --tab -- rosrun flood_monitor drone10.py

sleep 30

gnome-terminal --tab -- rosrun flood_monitor get_pose_2021.py