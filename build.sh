#!/bin/bash

# cwd=$(pwd)
# 
# # build the SLAM engine
# cd thirdparty/slam_engine/ORB_SLAM2/
# sh build.sh
# cd $cwd

# build the wrapper library
rm -rf build/
mkdir build
cd build
cmake ..
make
