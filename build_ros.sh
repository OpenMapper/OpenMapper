#!/bin/bash

root_dir=$(pwd)

export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:$root_dir

# build the wrapper library
rm -rf build/
mkdir build
cd build
cmake .. -DBUILD_ROS_BINDING=TRUE -DBUILD_TESTS=FALSE -DBUILD_APP=TRUE
make -j
