# hokuyo_node2
This package is to connect hokuyo via USB


Requirements
 - ruby

need package urg_c: https://github.com/ros-drivers/urg_c.git


Then build the package using colcon build
obs: In case the node does not read the hokuyo. You need to add the permission:
sudo chmod 666 /dev/ttyACM0
