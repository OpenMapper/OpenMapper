#!/bin/bash

# Build OpenMapper Static Lib only
mkdir -p build
cd build
cmake ..  -DBUILD_TESTS=TRUE 
make -j8
