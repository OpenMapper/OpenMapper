// (c) 2017 OpenMapper

#ifndef INCLUDE_OPENMAPPER_INPUT_SOURCE_H_
#define INCLUDE_OPENMAPPER_INPUT_SOURCE_H_

#include <iostream>
#include <thread>

// ORB_SLAM2
#include <System.h>

#include <opencv2/core/core.hpp>

namespace openmapper_wrapper {

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
    is_input_mode_set_ = true;
    source_ = source;

    switch (source_) {
      case kCamera: {
        camera_device_number_ = device;
        break;
      }
      case kFile: {
        path_to_file_ = device;
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

  void StreamVideo() {
    cv::VideoCapture cap;
    if (getInput() == InputSource::kCamera) {
      cap.open(std::stoi(camera_device_number_));
    } else if (getInput() == InputSource::kFile) {
      cap.open(path_to_file_);
    }

    if (!cap.isOpened()) {
      // Check if we succeeded.
      std::cerr << "ERROR: Camera input is broken!" << std::endl;
      return;
    }

    fps_ = cap.get(CV_CAP_PROP_FPS);
    assert(fps_ <= 1);

    while (true) {
      cap >> current_image_;
      current_image_time_sec_ = cap.get(CV_CAP_PROP_POS_MSEC) / 1000.0;
      auto t = std::chrono::duration<double>(1.0 / fps_);
      this_thread::sleep_for(t);
      if (current_image_.empty()) {
        std::cout << std::endl
                  << "Failed to load another image, assuming the video stream "
                     "has finished..."
                  << std::endl;

        break;
      }
    }
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
};

}  // namespace openmapper_wrapper

#endif  // INCLUDE_OPENMAPPER_INPUT_SOURCE_H_
