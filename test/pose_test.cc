// (c) 2017 OpenMapper

#include "gtest/gtest.h"

#include <string>
#include <vector>

#include "openmapper/openmapper.h"
#include "openmapper/settings.h"

namespace openmapper {

TEST(GetInitialPose, test_with_static_data) {
  std::vector<std::string> flags;
  std::shared_ptr<std::vector<double>> pos(new std::vector<double>);
  std::shared_ptr<std::vector<double>> rot(new std::vector<double>);

  flags.push_back(path_to_vocabulary);
  flags.push_back(path_to_settings);
  OpenMapper openmapper_engine_(flags);
  std::shared_ptr<openmapper::InputSource> input_source_(
      new openmapper::InputSource);

  input_source_->setInput(openmapper::InputSource::kFile, static_video);

  while (true) {
    input_source_->grabImage();

    bool tracking =
        openmapper_engine_.trackImage(input_source_->getCurrentImage(),
                                      input_source_->getCurrentImageTimeSec());
    if (!tracking) {
      break;
    }
    sleep(1 / input_source_->fps_);
  }

  sleep(1);
  openmapper_engine_.getPose(pos, rot);

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

}  // namespace openmapper
