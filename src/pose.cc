// (c) 2017 OpenMapper

#include "openmapper/pose.h"

namespace openmapper {

Pose::Pose() {
  cam_pose.camera_pos.resize(3, 0.0);
  cam_pose.camera_rot.resize(4, 0.0);
  cam_pose.camera_rot[3] = 1.0;
}

}  // namespace openmapper
