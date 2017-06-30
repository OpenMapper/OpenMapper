#!/bin/bash

root_dir=$(pwd)/../..

export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:$root_dir

# Build OpenMapper ROS Node
mkdir -p ros_build
cd build
cmake ../wrapper/ros/  -DCMAKE_BUILD_TYPE=DEBUG 
make -j4
