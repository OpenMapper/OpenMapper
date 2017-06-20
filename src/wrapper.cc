// (c) 2017 OpenMapper

#include "../include/wrapper.h"

namespace openmapper_wrapper {

Wrapper::Wrapper(const std::vector<std::string>& flags)
    : slam_engine(flags[0], flags[1], ORB_SLAM2::System::MONOCULAR, true) {}

Wrapper::~Wrapper() {}

int Wrapper::StartSLAM(const VideoSource source) {
  cv::Mat image;
  cv::Mat cam_transformation;

  if (source == kCamera) {
    cv::VideoCapture cap(0);
    // Open the default camera.
    if (!cap.isOpened()) {
      // Check if we succeeded.
      return -1;
    }

    while (true) {
      // Get a new frame from camera.
      cap >> image;

      time_t seconds = time(NULL);
      double tframe = seconds;

      if (image.empty()) {
        cerr << endl << "Failed to load image" << endl;
        return 1;
      }

      // Pass the image to the SLAM system.
      cam_transformation = slam_engine.TrackMonocular(image, tframe);
      std::cout << "Camera transformation: \n"
                << cam_transformation << std::endl;
      sleep(0.001);
    }
  }

  if (source == kFile) {
    // TODO(gocarlos): implement this.
  }

  return 0;
}

void Wrapper::StopSLAM() {
  // Stop all threads
  slam_engine.Shutdown();
}

void Wrapper::GetPose(std::vector<double>& pos, std::vector<double>& rot) {
  pos.resize(3);
  rot.resize(4);
  pos = {0.0, 0.0, 0.0};
  rot = {0.0, 0.0, 0.0, 1.0};
}
}
// namespace openmapper_wrapper
