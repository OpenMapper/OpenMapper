// (c) 2017 OpenMapper

#include "openmapper/openmapper.h"

namespace openmapper {

OpenMapper::OpenMapper(int argc, char** argv)
    : path_to_vocabulary_(argv[1]),
      path_to_settings_(argv[2]),
      slam_engine(argv[1], argv[2], ORB_SLAM2::System::MONOCULAR, false),
      input_source_(),
      pose_() {
  initialize();
}

OpenMapper::OpenMapper(const std::vector<std::string>& flags)
    : slam_engine(flags[0], flags[1], ORB_SLAM2::System::MONOCULAR, false),
      path_to_vocabulary_(flags[0]),
      path_to_settings_(flags[1]),
      input_source_(),
      pose_() {
  initialize();
}

void OpenMapper::initialize() {
  std::cout << "\n"
            << "Flags: "
            << "\n"
            << path_to_vocabulary_ << "\n"
            << path_to_settings_ << std::endl
            << std::endl;
}

OpenMapper::~OpenMapper() {}

bool OpenMapper::trackImage() {
  //  CHECK(input_source_.isIsInputModeSet())

  input_source_.grabImage();

  if (input_source_.getCurrentImage().empty()) {
    std::cout << std::endl
              << "Failed to load another image, assuming the video stream "
                 "has finished..."
              << std::endl;
    return false;
  } else {
    pose_.curr_cam_transformation =
        slam_engine.TrackMonocular(input_source_.getCurrentImage(),
                                   input_source_.getCurrentImageTimeSec());

    if (pose_.curr_cam_transformation.rows > 1) {
      std::cout << "Camera transformation at time " << std::setprecision(20)
                << input_source_.getCurrentImageTimeSec() << "\n"
                << pose_.curr_cam_transformation << std::endl;
      pose_.cam_pose.camera_pos[0] =
          pose_.curr_cam_transformation.at<float>(0, 3);
      pose_.cam_pose.camera_pos[1] =
          pose_.curr_cam_transformation.at<float>(1, 3);
      pose_.cam_pose.camera_pos[2] =
          pose_.curr_cam_transformation.at<float>(2, 3);

    } else {
      std::cout << "\r"
                << "Waiting for tracking: " << std::setprecision(20)
                << input_source_.getCurrentImageTimeSec() << " ";
    }
  }
  return true;
}

void OpenMapper::stopSLAM() {
  // Stop all threads
  slam_engine.Shutdown();
}

void OpenMapper::getPose(std::shared_ptr<std::vector<double>> pos,
                         std::shared_ptr<std::vector<double>> rot) {
  //	CHECK(pos,rot);
  *pos = pose_.cam_pose.camera_pos;
  *rot = pose_.cam_pose.camera_rot;
}
}
// namespace openmapper
