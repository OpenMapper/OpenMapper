// (c) 2017 OpenMapper

#ifndef INCLUDE_OPENMAPPER_INPUT_SOURCE_H_
#define INCLUDE_OPENMAPPER_INPUT_SOURCE_H_

#include <iostream>
#include <thread>
#include <glog/logging.h>

// ORB_SLAM2
#include <System.h>

#include <opencv2/core/core.hpp>

#include "openmapper/common.h"

namespace openmapper {

class InputSource {
 public:
  //
  // Input sensor
  //
  enum VideoSource {
    kCamera = 0,  // OpenCV streams from a camera
    kFile = 1,    // OpenCV streams from a movie file.
    kImage = 2    // The cv::Mat is directly accessed from another place, e.g.
                  // ROS, native mobile framework...
  };
  InputSource();
  virtual ~InputSource();

  cv::Mat getCurrentImage() { return current_image_; }
  void setCurrentImage(cv::Mat currentImage) { current_image_ = currentImage; }

  double getCurrentImageTimeSec() const { return current_image_time_sec_; }
  void setCurrentImageTimeSec(double currentImageTimeSec) {
    current_image_time_sec_ = currentImageTimeSec;
  }

  bool isIsInputModeSet() const { return is_input_mode_set_; }

  void setInput(VideoSource source, std::string device) {
    CHECK(device != "");
    LOG(INFO)    <<"Input device is set to: " << device;

    is_input_mode_set_ = true;
    source_ = source;

    switch (source_) {
      case kCamera: {
        camera_device_number_ = device;
        assert(camera_device_number_ != "");
        cap.open(std::stoi(camera_device_number_));
        break;
      }
      case kFile: {
        path_to_file_ = device;
        assert(path_to_file_ != "" && "Path to movie file must not be empty!");
        std::cout << "Path to file is: " << path_to_file_ << std::endl;
        cap.open(path_to_file_);
        break;
      }
      case kImage: {
        break;
      }
      default: {
        std::cerr << "ERROR: No Input set!" << std::endl;
        exit(1);
      }
    }
  }
  InputSource::VideoSource getInput() { return source_; }

  void grabImage() {
    assert(is_input_mode_set_ && "Input mode must be set!");

    if (!cap.isOpened()) {
      // Check if we succeeded.
      std::cerr << "ERROR: Camera input is broken!" << std::endl;
      return;
    }

    fps_ = cap.get(CV_CAP_PROP_FPS);
    assert(fps_ >= 1);

    cap >> current_image_;
    // FIXME: there is a problem with openCV returning 0 instead of the right
    // value. Using the current time instead.
    // current_image_time_sec_ = cap.get(CV_CAP_PROP_POS_MSEC) / 1000.0;
    Common::getCurrTimeSec(current_image_time_sec_);
  }

  float fps_;

 private:
  bool is_input_mode_set_;

  std::string path_to_file_;
  std::string camera_device_number_;

  //
  // OpenCV Matrix containing the current image.
  //
  cv::Mat current_image_;

  double current_image_time_sec_;
  VideoSource source_;

  cv::VideoCapture cap;
};

}  // namespace openmapper_wrapper

#endif  // INCLUDE_OPENMAPPER_INPUT_SOURCE_H_
