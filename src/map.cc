// (c) 2017 OpenMapper

#include "openmapper/map.h"

namespace openmapper {

Map::Map() {}

std::vector<cv::Point3f> Map::getFeaturesPosition() {
  CHECK_NOTNULL(slam_engine_.get());

  //   Get features collected by the low level engine.
  std::vector<ORB_SLAM2::MapPoint*> map_points =
      slam_engine_->mpTracker->mpMap->GetAllMapPoints();
  std::vector<cv::Point3f> all_map_points;

  for (size_t i = 0u; i < map_points.size(); ++i) {
    if (map_points[i] && !map_points[i]->isBad()) {
      cv::Point3f pos = cv::Point3f(map_points[i]->GetWorldPos());
      all_map_points.push_back(pos);
    }
  }
  return all_map_points;
}

}  // namespace openmapper
