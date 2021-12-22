#!/bin/bash

cd PX4-Autopilot
DONT_RUN=1 make px4_sitl gazebo_rover
cd ..

cd catkin_ws
catkin_make
cd ..

source ./setup.bash
#roslaunch rover mavros_posix_sitl.launch rviz:='true' x:=-9.88715 y:=8.23526 z:=0.411919 R:=-0.001258 P:=-0.010927 Y:=2.4979
#roslaunch rover mavros_posix_sitl.launch rviz:='true' x:=7.58439 y:=-102.82 z:=0.02 R:=0 P:=0 Y:=0
if [ "$1" == "" ]; then
    roslaunch rover mavros_posix_sitl.launch rviz:='true' x:=0 y:=0 z:=0.179517 R:=0 P:=0 Y:=0
else
    roslaunch rover mavros_posix_sitl.launch world_name:=$1 rviz:='true' x:=0 y:=0 z:=0.179517 R:=0 P:=0 Y:=0
fi