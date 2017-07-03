// (c) 2017 OpenMapper

#include "openmapper/common.h"

namespace openmapper {

Common::Common() {}

void Common::getCurrTimeSec(double& time) {
  time = std::chrono::time_point_cast<std::chrono::milliseconds>(
             std::chrono::system_clock::now())
             .time_since_epoch()
             .count() /
         1000.0;
}
} // namespace openmapper
