/*
 * input_source.cc
 *
 *  Created on: Jun 27, 2017
 *      Author: gocarlos
 */

#include <openmapper/input_source.h>

namespace openmapper_wrapper {

InputSource::InputSource()
    : is_input_mode_set_(false), current_image_time_sec_(0.0) {
  //  current_image_ = new cv::Mat;
  //  current_image_time_sec_ = new double;
}

InputSource::~InputSource() {
  //	delete current_image_, current_image_time_sec_;
}

}  // namespace openmapper_wrapper
