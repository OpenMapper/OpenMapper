#!/bin/bash

# Generate OpenMapper Xcode Framework Project for iOS
mkdir -p fw_build
rm -r ./fw_build/*
cd fw_build
cmake ../wrapper/ios -GXcode
