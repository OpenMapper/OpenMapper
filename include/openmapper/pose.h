// (c) 2017 OpenMapper

#ifndef INCLUDE_OPENMAPPER_POSE_H_
#define INCLUDE_OPENMAPPER_POSE_H_

#include <opencv2/core/core.hpp>

namespace openmapper {

class Pose {
 public:
  // TODO(gocarlos): finish this.
  Pose();
  //
  // OpenCV Matrix containing the current camera transformation from the origin
  // to the current pose.
  //
  cv::Mat curr_cam_transformation;

  struct CameraPose {
    // TODO(gocarlos): define camera_pos and camera_rot.
    //
    //
    //
    std::vector<double> camera_pos;

    //
    //
    //
    std::vector<double> camera_rot;
  } cam_pose;
};

}  // namespace openmapper

#endif  // INCLUDE_OPENMAPPER_POSE_H_
