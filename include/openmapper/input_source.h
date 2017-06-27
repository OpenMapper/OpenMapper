// (c) 2017 OpenMapper

#ifndef INCLUDE_OPENMAPPER_INPUT_SOURCE_H_
#define INCLUDE_OPENMAPPER_INPUT_SOURCE_H_

#include <opencv2/core/core.hpp>

namespace openmapper_wrapper {

class InputSource {
 public:
  InputSource();
  virtual ~InputSource();

  cv::Mat* current_image;
  double* current_image_time_sec;
};

}  // namespace openmapper_wrapper

#endif  // INCLUDE_OPENMAPPER_INPUT_SOURCE_H_
