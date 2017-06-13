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

	Wrapper wrapper;

	wrapper.StartSLAM(flags);
	wrapper.GetPose(pos, rot);

	double pose_error = 0.0;
	for (auto v : pos) {
		pose_error += v;
	}
	double maximal_error = 0.1;
	EXPECT_LT(pose_error, maximal_error);
}

}  // namespace openmapper_wrapper
