#!/bin/bash
source ../devel/setup.bash

echo "123456" | sudo -S chmod 777 /dev/tty* 

# roslaunch mavros px4.launch  & sleep 3

rosrun mavros mavcmd long 511 31 10000 0 0 0 0 0;

roslaunch vrpn_client_ros sample.launch server:=10.1.1.198 & sleep 3

roslaunch odom_converter converter.launch & sleep 3

roslaunch ekf nokov.launch & sleep 3

roslaunch px4ctrl run_ctrl.launch 