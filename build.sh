#!/bin/bash

# build the wrapper library
rm -rf build/
mkdir build
cd build
cmake ..
make -j
