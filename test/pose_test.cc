// (c) 2017 OpenMapper

#include "gtest/gtest.h"

#include <string>
#include <vector>

#include "config.h"
#include "openmapper/wrapper.h"

namespace openmapper_wrapper {

TEST(GetInitialPose, test_with_static_data) {
  std::vector<std::string> flags;
  std::shared_ptr<std::vector<double>> pos(new std::vector<double>);
  std::shared_ptr<std::vector<double>> rot(new std::vector<double>);

  flags.push_back(path_to_vocabulary);
  flags.push_back(path_to_settings);
  Wrapper wrapper(flags);

  wrapper.StartSLAM();
  sleep(1);
  wrapper.GetPose(pos, rot);

  // Get the distance traveled by the camera between start point and end point.
  double distance = 0.0;
  for (auto v : *pos) {
    distance += std::pow(v, 2);
    std::cout << "v: " << v << std::endl;
  }
  sqrt(distance);

  std::cout << "The distance is: " << distance << std::endl;
  // wrapper.StopSLAM();

  double maximal_distance = 0.1;
  EXPECT_LE(distance, maximal_distance);
}

// FIXME(gocarlos): there is a bug in the software: if the tests are run after
// each other the second will fail.
// Eventually is something wrongly initialized.

// TEST(GetInitialPose, test_with_dynamic_data) {
//	std::vector<std::string> flags;
//	std::shared_ptr<std::vector<double>> pos(new std::vector<double>);
//	std::shared_ptr<std::vector<double>> rot(new std::vector<double>);
//
//	flags.push_back(path_to_vocabulary);
//	flags.push_back(path_to_settings);
//	Wrapper wrapper(flags);
//
//	wrapper.StartSLAM(Wrapper::VideoSource::kFile, dynamic_video);
//	sleep(1);
//	wrapper.GetPose(pos, rot);
//
//	// Get the distance traveled by the camera between start point and end
// point.
//	double distance = 0.0;
//	for (auto v : *pos) {
//		distance += std::pow(v, 2);
//		std::cout << "v: " << v << std::endl;
//	}
//	sqrt(distance);
//
//	std::cout << "The distance is: " << distance << std::endl;
//	wrapper.StopSLAM();
//
//	double minimal_distance = 0.1;
//	EXPECT_GE(distance, minimal_distance);
//}

}  // namespace openmapper_wrapper
