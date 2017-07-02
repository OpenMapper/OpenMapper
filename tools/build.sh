#!/bin/bash

# Build OpenMapper Static Lib only
mkdir -p build
cd build
cmake ..  -DBUILD_TESTS=TRUE -DGLOG_ROOT_DIR=../../thirdparty/glog
make -j8
