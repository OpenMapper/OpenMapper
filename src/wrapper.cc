// (c) 2017 OpenMapper

#include "../include/wrapper.h"

namespace openmapper_wrapper {

Wrapper::Wrapper(const std::vector<std::string>& flags)
    : slam_engine(flags[0], flags[1], ORB_SLAM2::System::MONOCULAR, false),
      is_tracking(false),
      curr_frame_time_stamp(0.0) {
  std::cout << "Flags: "
            << "\n"
            << flags[0] << "\n"
            << flags[1] << std::endl
            << std::endl;

  cam_pose.camera_pos.resize(3);
  cam_pose.camera_rot.resize(4);
}

Wrapper::~Wrapper() {}

int Wrapper::StartSLAM(const VideoSource source, const std::string input_file) {
  cv::VideoCapture cap;
  assert(input_file == "");
  //  cv::namedWindow("MyWindow", CV_WINDOW_AUTOSIZE); //create a window with
  //  the name "MyWindow"
  time_t start_time_seconds = time(NULL);

  if (source == kCamera) {
    cap.open(0);
  } else if (source == kFile) {
    cap.open(input_file);
  }

  // Open the default camera.
  if (!cap.isOpened()) {
    // Check if we succeeded.
    std::cerr << "Camera input is broken!" << std::endl;
    return -1;
  }
  float fps = cap.get(CV_CAP_PROP_FPS);
  assert(fps <= 1);

  double start_time_stap;
  GetCurrTimeSec(start_time_stap);
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

      return 0;
    }

    // Pass the image to the SLAM system.
    curr_cam_transformation =
        slam_engine.TrackMonocular(curr_image, curr_frame_time_stamp);

    GetCurrTimeSec(curr_time_stamp);
    time_diff = curr_time_stamp - start_time_stap;
    start_time_stap = curr_time_stamp;

    // Wait for the next frame or take the new one of we take longer then 1/fps
    // to track the image.
    double time_to_wait = std::max(1.0 / fps - time_diff, 0.0);

    if (curr_cam_transformation.rows > 1) {
      is_tracking = true;
      std::cout << "Camera transformation at time " << std::setprecision(20)
                << curr_frame_time_stamp << "\n"
                << curr_cam_transformation << std::endl;
      cam_pose.camera_pos[0] = curr_cam_transformation.at<float>(0, 3);
      cam_pose.camera_pos[1] = curr_cam_transformation.at<float>(1, 3);
      cam_pose.camera_pos[2] = curr_cam_transformation.at<float>(2, 3);

    } else {
      std::cout << "\r"
                << "Waiting for tracking: " << std::setprecision(20)
                << curr_frame_time_stamp;
    }

    sleep(time_to_wait);
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
  if (!is_tracking) {
    pos = {0.0, 0.0, 0.0};
    rot = {0.0, 0.0, 0.0, 1.0};
  } else {
    pos = cam_pose.camera_pos;
    rot = cam_pose.camera_rot;
  }
}

void Wrapper::DebugInfo() {
  std::cout << "Camera transformation at time " << std::setprecision(20)
            << curr_frame_time_stamp << "\n"
            << curr_cam_transformation << std::endl;
}

void Wrapper::GetCurrTimeSec(double& time) {
  time = std::chrono::time_point_cast<std::chrono::milliseconds>(
             std::chrono::system_clock::now())
             .time_since_epoch()
             .count() /
         1000.0;
}
}
// namespace openmapper_wrapper
