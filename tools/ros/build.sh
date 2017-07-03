#!/bin/bash

root_dir=$(pwd)/../..

export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:$root_dir

# Build OpenMapper ROS Node
mkdir -p ros_build
cd ros_build
cmake ../wrapper/ros/  -DCMAKE_BUILD_TYPE=DEBUG -DBUILD_TESTS=TRUE
make -j4
