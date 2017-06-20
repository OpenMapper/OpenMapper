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

	wrapper.StartSLAM(Wrapper::VideoSource::kFile, input_settings);
	wrapper.GetPose(pos, rot);

	double pose_error = 0.0;
	for (auto v : pos) {
		pose_error += v;
	}
	double maximal_error = 0.1;
	EXPECT_LT(pose_error, maximal_error);
}

}  // namespace openmapper_wrapper
