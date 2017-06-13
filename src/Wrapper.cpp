/*
 * Wrapper.cpp
 *
 *  Created on: Jun 13, 2017
 *      Author: gocarlos
 */

#include "../include/Wrapper.hpp"

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
/* namespace openmapper_wrapper */
