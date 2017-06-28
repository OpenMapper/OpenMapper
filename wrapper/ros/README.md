##ROS Node Usage##

rosrun OpenMapper OpenMapperAppROSNode path_to_vocabulary path_to_settings [opencv_live  0| opencv_movie /home/user0/movie.mov | ros_topic "/camera/image_raw" ] 


Example: 

// // TODO(gocarlos): the path to the files should be the full path, when debugging
// // with dbg the relative path does not work.
// std::string path_to_vocabulary =
//     "../thirdparty/slam_engine/ORB_SLAM2/Vocabulary/ORBvoc.txt";
// std::string path_to_settings =
//     "../thirdparty/slam_engine/ORB_SLAM2/Vocabulary/iphone.yaml";
// 
// std::string static_video = "../test/test_data/static.mov";
// std::string dynamic_video = "../test/test_data/dynamic.mov";



rosrun OpenMapper OpenMapperAppROSNode /home/gocarlos/Dropbox/software/openmapper/OpenMapper/thirdparty/slam_engine/ORB_SLAM2/Vocabulary/ORBvoc.txt /home/gocarlos/Dropbox/software/openmapper/OpenMapper/thirdparty/slam_engine/ORB_SLAM2/Vocabulary/iphone.yaml opencv_live  0 
