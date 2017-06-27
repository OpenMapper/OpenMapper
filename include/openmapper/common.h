// (c) 2017 OpenMapper

#ifndef INCLUDE_OPENMAPPER_COMMON_H_
#define INCLUDE_OPENMAPPER_COMMON_H_

#include <chrono>

namespace openmapper_wrapper {

class Common {
 public:
  Common();
  virtual ~Common();
  static void GetCurrTimeSec(double& time);
};

}  // namespace openmapper_wrapper

#endif  // INCLUDE_OPENMAPPER_COMMON_H_
