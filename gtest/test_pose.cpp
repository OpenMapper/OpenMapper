//============================================================================
// Name        : test_pose.cpp
// Author      : Carlos Gomes
// Version     :
// Copyright   : Copyright OpenMapper
// Description : Unit test for testing the pose results
//============================================================================

#include "gtest/gtest.h"

#include <string>
#include <vector>

#include "../include/Wrapper.hpp"

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

int main(int argc, char** argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}

}  // namespace openmapper_wrapper
