// (c) 2017 OpenMapper

#include "gtest/gtest.h"

#include <string>
#include <vector>

#include "wrapper.h"
#include "config.h"

namespace openmapper_wrapper {

TEST(GetInitialPose, test_with_static_data) {
  std::vector<std::string> flags;
  std::vector<double> pos;
  std::vector<double> rot;

  flags.push_back(path_to_vocabulary);
  flags.push_back(path_to_settings);
  Wrapper wrapper(flags);

  wrapper.StartSLAM(Wrapper::VideoSource::kFile, static_video);
  wrapper.GetPose(pos, rot);

  double distance = 0.0;
  for (auto v : pos) {
    distance += std::pow(v, 2);
    std::cout << "v: " << v << std::endl;
  }
  sqrt(distance);
  std::cout << "The distance is: " << distance << std::endl;
  double maximal_distance = 0.1;
  EXPECT_LE(distance, maximal_distance);
}

TEST(GetInitialPose, test_with_dynamic_data) {
	std::vector<std::string> flags;
	std::vector<double> pos;
	std::vector<double> rot;

	flags.push_back(path_to_vocabulary);
	flags.push_back(path_to_settings);
	Wrapper wrapper(flags);

	wrapper.StartSLAM(Wrapper::VideoSource::kFile, dynamic_video);
	wrapper.GetPose(pos, rot);

	double distance = 0.0;
	for (auto v : pos) {
		distance += std::pow(v,2);
		std::cout<<"v: "<<v<<std::endl;
	}
	sqrt(distance);
	std::cout <<"The distance is: "<<distance<<std::endl;

	double minimal_distance = 0.1;
	EXPECT_GE(distance, minimal_distance);
}


}  // namespace openmapper_wrapper
