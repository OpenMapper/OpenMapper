//============================================================================
// Name        : wrapper.cc
// Author      : Carlos Gomes
// Copyright   : Copyright OpenMapper
// Description : Main class of the wrapper.
//============================================================================

#include "../include/wrapper.h"

namespace openmapper_wrapper {

Wrapper::Wrapper() {
	// TODO Auto-generated constructor stub

}

Wrapper::~Wrapper() {
	// TODO Auto-generated destructor stub
}

void Wrapper::StartSLAM(const std::vector<std::string> &flags) {

}

void Wrapper::GetPose(std::vector<double> &pos, std::vector<double> &rot) {
	pos.resize(3);
	rot.resize(4);
	pos= {0.0,0.0,0.0};
	rot= {0.0,0.0,0.0,1.0};
}

}
 // namespace openmapper_wrapper
