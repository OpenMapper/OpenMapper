#!/bin/bash

rm -rf build/
# rm -rf bin/
# rm -rf lib/

# Build OpenMapper Static Lib only
mkdir -p build
cd build
cmake ..  -DBUILD_TESTS=TRUE
make -j8
