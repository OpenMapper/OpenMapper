#!/bin/bash

# Settings
root_dir=$(pwd)
#TODO(gocarlos): add here a proper download link to GIT LFS
address_test_data="https://www.dropbox.com/s/tdogcjoo1cgvt38/test_data.tar.gz?raw=1"


# Unpack the Vocabulary file.
cd thirdparty/slam_engine/ORB_SLAM2/Vocabulary/
tar -xf ORBvoc.txt.tar.gz
cd $root_dir

# Download the test data from the web.
cd test
wget $address_test_data
mv test_data.tar.gz?raw=1 test_data.tar.gz
tar -xf test_data.tar.gz
cd ..

# Run the tests, if running with debug settings, then take the *_d executable. 
cd bin
FILE="OpenMapperTests"
if [ -f $FILE ]; then
   echo "File $FILE exists."
   ./OpenMapperTests
else
   echo "File $FILE does not exist."
   ./OpenMapperTests_d   
fi

