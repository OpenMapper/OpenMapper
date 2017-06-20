// (c) 2017 OpenMapper

#ifndef TEST_CONFIG_H_
#define TEST_CONFIG_H_

#include <string>

// TODO(gocarlos): add some test data which can be executed on travis.
std::string path_to_vocabulary =
		"../thirdparty/slam_engine/ORB_SLAM2/Vocabulary/ORBvoc.txt";
std::string path_to_settings =
		"../thirdparty/slam_engine/ORB_SLAM2/Vocabulary/webcam.yaml";
std::string input_settings = "../test/test_data/test_video.mov";

#endif // TEST_CONFIG_H_
