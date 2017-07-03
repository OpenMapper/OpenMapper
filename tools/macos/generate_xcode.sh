#!/bin/bash

# Generate OpenMapper Framework Project
mkdir -p xc_build
cd xc_build
cmake ../. -GXcode
