#include "frame.h"
#include "pose.h"

class SlamEngine {
 public:
  void processFrame(const Frame &frame, Pose* pose);
 private:
  Pose last_pose;
}
