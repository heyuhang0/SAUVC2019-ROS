#!/bin/bash

cd ~/catkin_ws/src; 
echo 'bubbles_control' | xargs tar cf - | ssh $1 tar xf - -C /home/$2/catkin_ws/src/;
cd -;
