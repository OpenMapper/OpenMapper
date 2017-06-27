// (c) 2017 OpenMapper

#include "openmapper/wrapper.h"

namespace openmapper_wrapper {

Wrapper::Wrapper(int argc, char** argv)
    : path_to_vocabulary(argv[1]),
      path_to_settings(argv[2]),
      slam_engine(argv[1], argv[2], ORB_SLAM2::System::MONOCULAR, false),
      input_source() {
  Initialize();
}

Wrapper::Wrapper(const std::vector<std::string>& flags)
    : slam_engine(flags[0], flags[1], ORB_SLAM2::System::MONOCULAR, false),
      path_to_vocabulary(flags[0]),
      path_to_settings(flags[1]),
      input_source() {
  Initialize();
}

void Wrapper::Initialize() {
  has_tracked = false;
  curr_frame_time_stamp = 0.0;
  cam_pose.camera_pos.resize(3, 0);
  cam_pose.camera_rot.resize(4, 0);
  std::cout << "\n"
            << "Flags: "
            << "\n"
            << path_to_vocabulary << "\n"
            << path_to_settings << std::endl
            << std::endl;
}

Wrapper::~Wrapper() {}

int Wrapper::StartSLAM(const VideoSource source, const std::string input_file) {
  cv::VideoCapture cap;
  assert(input_file != "");

  if (source == kCamera) {
    // Open the default camera.
    cap.open(0);
  } else if (source == kFile) {
    cap.open(input_file);
  }

  if (!cap.isOpened()) {
    // Check if we succeeded.
    std::cerr << "Camera input is broken!" << std::endl;
    return -1;
  }

  float fps = cap.get(CV_CAP_PROP_FPS);
  assert(fps >= 1);

  double start_time_stap;
  double time_to_wait;
  Common::GetCurrTimeSec(start_time_stap);
  double curr_time_stamp = start_time_stap;
  while (true) {
    // Get a new frame from camera.
    cap >> curr_image;
    curr_frame_time_stamp = cap.get(CV_CAP_PROP_POS_MSEC) / 1000.0;
    double time_diff = curr_time_stamp - curr_frame_time_stamp;

    if (curr_image.empty()) {
      std::cout << std::endl
                << "Failed to load another image, assuming the video stream "
                   "has finished..."
                << std::endl;

      break;
    }

    // Pass the image to the SLAM system.
    curr_cam_transformation =
        slam_engine.TrackMonocular(curr_image, curr_frame_time_stamp);

    Common::GetCurrTimeSec(curr_time_stamp);
    time_diff = curr_time_stamp - start_time_stap;
    start_time_stap = curr_time_stamp;

    // Wait for the next frame or take the new one of we take longer then 1/fps
    // to track the image.
    time_to_wait = std::max(1.0 / fps - time_diff, 0.0);

    if (curr_cam_transformation.rows > 1) {
      has_tracked = true;
      //      std::cout << "Camera transformation at time " <<
      //      std::setprecision(20)
      //                << curr_frame_time_stamp << "\n"
      //                << curr_cam_transformation << std::endl;
      cam_pose.camera_pos[0] = curr_cam_transformation.at<float>(0, 3);
      cam_pose.camera_pos[1] = curr_cam_transformation.at<float>(1, 3);
      cam_pose.camera_pos[2] = curr_cam_transformation.at<float>(2, 3);

    } else {
      std::cout << "\r"
                << "Waiting for tracking: " << std::setprecision(20)
                << curr_frame_time_stamp << " ";
    }

    sleep(time_to_wait);
  }

  return 0;
}

void Wrapper::StopSLAM() {
  // Stop all threads
  slam_engine.Shutdown();
}

void Wrapper::GetPose(std::shared_ptr<std::vector<double>> pos,
                      std::shared_ptr<std::vector<double>> rot) {
  *pos = cam_pose.camera_pos;
  *rot = cam_pose.camera_rot;
}

void Wrapper::DebugInfo() {
  std::cout << "Camera transformation at time " << std::setprecision(20)
            << curr_frame_time_stamp << "\n"
            << curr_cam_transformation << std::endl;
}
}
// namespace openmapper_wrapper
