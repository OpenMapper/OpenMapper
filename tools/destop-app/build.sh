#!/bin/bash

# Build OpenMapper Desktop App

cd wrapper/desktop/
mkdir -p da_build
cd da_build
cmake .. -DCMAKE_BUILD_TYPE=DEBUG 
make -j4
