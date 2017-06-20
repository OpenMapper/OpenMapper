// (c) 2017 OpenMapper

#include "gtest/gtest.h"

#include <string>
#include <vector>

#include "wrapper.h"

namespace openmapper_wrapper {

// TODO (gocarlos) add here some data, with which the tests should be run.
std::vector<std::string> flags;
std::vector<double> pos;
std::vector<double> rot;

TEST(GetInitialPose, test_with_static_data) {

	// TODO(gocarlos): add some test data which can be executed on travis.
  std::string path_to_vocabulary;
  path_to_vocabulary =
      "../thirdparty/slam_engine/ORB_SLAM2/Vocabulary/ORBvoc.txt";
  std::string path_to_settings;
  path_to_settings =
      "../thirdparty/slam_engine/ORB_SLAM2/Vocabulary/webcam.yaml";

  flags.push_back(path_to_vocabulary);
  flags.push_back(path_to_settings);
  Wrapper wrapper(flags);

  // wrapper.StartSLAM(Wrapper::VideoSource::kCamera);
  wrapper.GetPose(pos, rot);

  double pose_error = 0.0;
  for (auto v : pos) {
    pose_error += v;
  }
  double maximal_error = 0.1;
  EXPECT_LT(pose_error, maximal_error);
}

}  // namespace openmapper_wrapper
