#!/bin/bash

# Build OpenMapper Desktop App
mkdir -p da_build
cd build
cmake ../wrapper/desktop-app  -DCMAKE_BUILD_TYPE=DEBUG 
make -j8
