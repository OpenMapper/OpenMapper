// (c) 2017 OpenMapper

#include <openmapper/input_source.h>

namespace openmapper {

InputSource::InputSource()
    : is_input_mode_set_(false),
      current_image_time_sec_(0.0),
      fps_(0.0),
      source_(InputSource::kCamera) {}

}  // namespace openmapper
