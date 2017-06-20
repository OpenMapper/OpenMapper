#!/bin/bash

# Unpack the Vocabulary file.
root_dir=$(pwd)
cd thirdparty/slam_engine/ORB_SLAM2/Vocabulary/
tar -xf ORBvoc.txt.tar.gz
cd $root_dir

# Run the tests.
cd bin
./OpenMapperTests
