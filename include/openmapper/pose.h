// (c) 2017 OpenMapper

#ifndef INCLUDE_OPENMAPPER_POSE_H_
#define INCLUDE_OPENMAPPER_POSE_H_

namespace openmapper_wrapper {

class Pose {
 public:
  // TODO(gocarlos): finish this.
  Pose() {
    cam_pose.camera_pos.resize(3, 0);
    cam_pose.camera_rot.resize(4, 0);
  }

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

}  // namespace openmapper_wrapper

#endif  // INCLUDE_OPENMAPPER_POSE_H_
