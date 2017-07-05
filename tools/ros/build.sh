#!/bin/bash

# This script should be executed from the root directory of the OpenMapper software. 

root_dir=$(pwd)
export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:$root_dir
# export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:/home/gocarlos/Dropbox/software/openmapper/OpenMapper

# Build OpenMapper ROS Node
cd wrapper/ros/
mkdir -p ros_build
cd ros_build
cmake .. -DCMAKE_BUILD_TYPE=DEBUG -DBUILD_TESTS=TRUE
make -j4
