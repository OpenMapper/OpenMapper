// (c) 2017 OpenMapper

#ifndef INCLUDE_OPENMAPPER_RENDERER_H_
#define INCLUDE_OPENMAPPER_RENDERER_H_

#include <memory>

#include <opencv2/core/core.hpp>

namespace openmapper {

class Renderer {
 public:
  //
  // Constructor.
  //
  Renderer();

  //
  // Display the current image into the screen over openGL.
  //
  void displayImage(const cv::Mat& img);

  //
  // Display the tracked features into the screen over openGL.
  //
  void displayFeatures(const std::vector<cv::Point3f>& all_map_points);

  //
  // Project points into the screen.
  //
  void projectPoints(
      const std::vector<cv::Point3f>& all_map_points,
      std::shared_ptr<std::vector<cv::Point2f>> projected_points);
};

}  // namespace openmapper

#endif  // INCLUDE_OPENMAPPER_RENDERER_H_
