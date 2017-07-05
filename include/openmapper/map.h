// (c) 2017 OpenMapper

#ifndef INCLUDE_OPENMAPPER_MAP_H_
#define INCLUDE_OPENMAPPER_MAP_H_

#include <glog/logging.h>
#include <opencv2/core/core.hpp>

// ORB_SLAM2
#include <System.h>

namespace openmapper {

class Map {
 public:
  Map();

  std::vector<cv::Point3f> getFeaturesPosition();
  std::shared_ptr<ORB_SLAM2::System> slam_engine_;
};

}  // namespace openmapper

#endif  // INCLUDE_OPENMAPPER_MAP_H_
