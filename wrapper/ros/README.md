# ROS Node Usage

rosrun open_mapper_ros open_mapper_ros path_to_vocabulary path_to_settings [ opencv_live 0 | opencv_movie PATH/movie.mov | ros_topic "/camera/image_raw" ]

## Example:

- `rosrun open_mapper_ros open_mapper_ros PATH/thirdparty/slam_engine/ORB_SLAM2/Vocabulary/ORBvoc.txt PATH/thirdparty/slam_engine/ORB_SLAM2/Vocabulary/iphone.yaml opencv_live 0`
- `roslaunch OpenMapper default.launch`
- `./tools/ros/run_ros.sh`
