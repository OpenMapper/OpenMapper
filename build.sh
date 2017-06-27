#!/bin/bash

# build the wrapper library
rm -rf build/
mkdir build
cd build
cmake ..  -DCMAKE_BUILD_TYPE=DEBUG 
make -j
