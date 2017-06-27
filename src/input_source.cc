/*
 * input_source.cc
 *
 *  Created on: Jun 27, 2017
 *      Author: gocarlos
 */

#include <openmapper/input_source.h>

namespace openmapper_wrapper {

InputSource::InputSource() {
  current_image = new cv::Mat;
  current_image_time_sec = new double;
}

InputSource::~InputSource() { delete current_image, current_image_time_sec; }

}  // namespace openmapper_wrapper
