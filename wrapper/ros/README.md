#ROS Node Usage

rosrun OpenMapper OpenMapperAppROSNode path_to_vocabulary path_to_settings [ opencv_live  0 | opencv_movie PATH/movie.mov | ros_topic "/camera/image_raw" ]


##Example:

* `rosrun OpenMapper OpenMapperAppROSNode PATH/thirdparty/slam_engine/ORB_SLAM2/Vocabulary/ORBvoc.txt PATH/thirdparty/slam_engine/ORB_SLAM2/Vocabulary/iphone.yaml opencv_live  0`
* `roslaunch OpenMapper default.launch`
