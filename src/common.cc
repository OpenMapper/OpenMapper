/*
 * input_source.cc
 *
 *  Created on: Jun 27, 2017
 *      Author: gocarlos
 */

#include "openmapper/common.h"

namespace openmapper_wrapper {

Common::Common() {}

Common::~Common() {}

void Common::GetCurrTimeSec(double& time) {
  time = std::chrono::time_point_cast<std::chrono::milliseconds>(
             std::chrono::system_clock::now())
             .time_since_epoch()
             .count() /
         1000.0;
}
} /* namespace openmapper_wrapper */
