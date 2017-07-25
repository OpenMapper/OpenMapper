#!/bin/bash

set -ev
# The -e flag causes the script to exit as soon as one command returns a non-zero exit code.
# The -v flag makes the shell print all lines in the script before executing them, which helps identify which steps failed.

rm -rf build/
rm -rf bin/
rm -rf lib/

# Build OpenMapper Static Lib only
mkdir -p build
cd build
cmake ..  -DBUILD_TESTS=TRUE -DCMAKE_BUILD_TYPE=Debug -DDO_COVERAGE_TEST=ON
make -j8
