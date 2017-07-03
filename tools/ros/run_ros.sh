#!/bin/bash

# This script should be executed from the root directory of the OpenMapper software. 
root_dir=$(pwd)
ros_core="roscore"
rviz_cmd="rosrun rviz rviz -d $root_dir/wrapper/ros/openmapper.rviz&"
node_cmd=".$root_dir/ros_build/devel/lib/open_mapper_ros/open_mapper_ros $root_dir/thirdparty/slam_engine/ORB_SLAM2/Vocabulary/ORBvoc.txt $root_dir/thirdparty/slam_engine/ORB_SLAM2/Vocabulary/iphone.yaml"


#eval $rviz_cmd
eval $node_cmd

