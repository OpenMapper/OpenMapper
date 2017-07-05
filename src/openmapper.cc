// (c) 2017 OpenMapper

#include "openmapper/openmapper.h"

#include <opencv2/core/eigen.hpp>
#include "Eigen/Core"

namespace openmapper {

OpenMapper::OpenMapper(const std::vector<std::string>& flags)
    : slam_engine_(new ORB_SLAM2::System(flags[0], flags[1],
                                         ORB_SLAM2::System::MONOCULAR, false)),
      path_to_vocabulary_(flags[0]),
      path_to_settings_(flags[1]),
      pose_(),
      map_(new Map) {
  initialize();
}

void OpenMapper::initialize() {
  LOG(INFO) << "Flags: "
            << "\n"
            << path_to_vocabulary_ << "\n"
            << path_to_settings_ << "\n\n";

  // Pass the slam_engine to the map in order to access all the features
  // contained in the slam map.
  map_->slam_engine_ = slam_engine_;
}

bool OpenMapper::trackImage(const cv::Mat& img, const double& time_stamp) {
  if (img.empty()) {
    LOG(INFO) << "Failed to load another image, assuming the video stream "
                 "has finished...";
    return false;
  } else {
    pose_.curr_cam_transformation =
        slam_engine_->TrackMonocular(img, time_stamp);

    if (pose_.curr_cam_transformation.rows > 1) {
      //      LOG(INFO) << "Camera transformation at time " <<
      //      std::setprecision(20)
      //                << time_stamp << "\n"
      //                << pose_.curr_cam_transformation;

      // Position
      pose_.cam_pose.camera_pos[0] =
          pose_.curr_cam_transformation.at<float>(0, 3);
      pose_.cam_pose.camera_pos[1] =
          pose_.curr_cam_transformation.at<float>(1, 3);
      pose_.cam_pose.camera_pos[2] =
          pose_.curr_cam_transformation.at<float>(2, 3);

      // Rotation
      Eigen::Matrix<float, 3, 3> b;
      cv2eigen(pose_.curr_cam_transformation.colRange(0, 3).rowRange(0, 3), b);
      Eigen::Quaternionf q(b);
      pose_.cam_pose.camera_rot[0] = q.x();
      pose_.cam_pose.camera_rot[1] = q.y();
      pose_.cam_pose.camera_rot[2] = q.z();
      pose_.cam_pose.camera_rot[3] = q.w();

    } else {
      std::cout << "\r"
                << "Waiting for tracking: " << std::setprecision(20)
                << time_stamp << " ";
    }
  }
  return true;
}

void OpenMapper::stopSLAM() {
  // Stop all threads
  slam_engine_->Shutdown();
}

void OpenMapper::getPose(std::shared_ptr<std::vector<double>> pos,
                         std::shared_ptr<std::vector<double>> rot) {
  CHECK(pos);
  CHECK(rot);
  *pos = pose_.cam_pose.camera_pos;
  *rot = pose_.cam_pose.camera_rot;
}
}
// namespace openmapper
