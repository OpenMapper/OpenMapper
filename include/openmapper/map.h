// (c) 2017 OpenMapper

#ifndef INCLUDE_OPENMAPPER_MAP_H_
#define INCLUDE_OPENMAPPER_MAP_H_

#include <opencv2/core/core.hpp>

namespace openmapper {

class Map {
 public:
  Map();

  void getFeatures();

 private:
  Map* orb_slam_map_;
};

}  // namespace openmapper

#endif  // INCLUDE_OPENMAPPER_MAP_H_
