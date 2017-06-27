// (c) 2017 OpenMapper

#include "openmapper/wrapper.h"

namespace openmapper_wrapper {

Wrapper::Wrapper(int argc, char** argv)
    : path_to_vocabulary_(argv[1]),
      path_to_settings_(argv[2]),
      slam_engine(argv[1], argv[2], ORB_SLAM2::System::MONOCULAR, false),
      input_source_(),
      pose_() {
  Initialize();
}

Wrapper::Wrapper(const std::vector<std::string>& flags)
    : slam_engine(flags[0], flags[1], ORB_SLAM2::System::MONOCULAR, false),
      path_to_vocabulary_(flags[0]),
      path_to_settings_(flags[1]),
      input_source_(),
      pose_() {
  Initialize();
}

void Wrapper::Initialize() {
  std::cout << "\n"
            << "Flags: "
            << "\n"
            << path_to_vocabulary_ << "\n"
            << path_to_settings_ << std::endl
            << std::endl;
}

Wrapper::~Wrapper() {}

int Wrapper::StartSLAM() {
  assert(!input_source_.isIsInputModeSet());

  double start_time_stap;
  Common::GetCurrTimeSec(start_time_stap);
  double curr_time_stamp = start_time_stap;
  std::thread image_getter(std::bind(&InputSource::StreamVideo, input_source_));

  while (true) {
    double curr_frame_time_stamp = input_source_.getCurrentImageTimeSec();
    double time_diff = curr_time_stamp - curr_frame_time_stamp;

    cv::Mat curr_image;
    curr_image = input_source_.getCurrentImage();

    if (curr_image.empty()) {
      std::cout << std::endl
                << "Failed to load another image, assuming the video stream "
                   "has finished..."
                << std::endl;

      break;
    }

    // Pass the image to the SLAM system.
    pose_.curr_cam_transformation =
        slam_engine.TrackMonocular(curr_image, curr_frame_time_stamp);

    Common::GetCurrTimeSec(curr_time_stamp);
    time_diff = curr_time_stamp - start_time_stap;
    start_time_stap = curr_time_stamp;

    // Wait for the next frame or take the new one of we take longer then 1/fps
    // to track the image.
    double time_to_wait = std::max(1.0 / input_source_.fps_ - time_diff, 0.0);

    if (pose_.curr_cam_transformation.rows > 1) {
      std::cout << "Camera transformation at time " << std::setprecision(20)
                << curr_frame_time_stamp << "\n"
                << pose_.curr_cam_transformation.rows << std::endl;
      pose_.cam_pose.camera_pos[0] =
          pose_.curr_cam_transformation.at<float>(0, 3);
      pose_.cam_pose.camera_pos[1] =
          pose_.curr_cam_transformation.at<float>(1, 3);
      pose_.cam_pose.camera_pos[2] =
          pose_.curr_cam_transformation.at<float>(2, 3);

    } else {
      std::cout << "\r"
                << "Waiting for tracking: " << std::setprecision(20)
                << curr_frame_time_stamp << " ";
    }
    auto t = std::chrono::duration<double>(time_to_wait);
    sleep(time_to_wait);
  }

  image_getter.join();

  return 0;
}

void Wrapper::StopSLAM() {
  // Stop all threads
  slam_engine.Shutdown();
}

void Wrapper::GetPose(std::shared_ptr<std::vector<double>> pos,
                      std::shared_ptr<std::vector<double>> rot) {
  *pos = pose_.cam_pose.camera_pos;
  *rot = pose_.cam_pose.camera_rot;
}

void Wrapper::DebugInfo() {
  //  std::cout << "Camera transformation at time " << std::setprecision(20)
  //            << curr_frame_time_stamp << "\n"
  //            << curr_cam_transformation << std::endl;
}
}
// namespace openmapper_wrapper
