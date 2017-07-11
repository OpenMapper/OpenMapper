// (c) 2017 OpenMapper

#ifndef INCLUDE_OPENMAPPER_RENDERER_H_
#define INCLUDE_OPENMAPPER_RENDERER_H_

#include <memory>

#include <opencv2/core/core.hpp>

#include "openmapper/openmapper.h"

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
  void displayImage(const cv::Mat &img);

  //
  // Display the tracked features into the screen over openGL.
  //
  void displayFeatures(const std::vector<cv::Point3f> &all_map_points);

  //
  // Project points into the screen.
  //
  void projectPoints(
      const std::vector<cv::Point3f> &all_map_points,
      std::shared_ptr<std::vector<cv::Point2f>> projected_points);

  //
  // Display the points which are being tracked.
  //
  void drawMapPoints();

  //
  // Display the current image.
  //
  void drawCurrentImage();

  //
  // Instance of the openmapper engine. Its here in order to access some
  // information from the low level engine.
  // TODO(gocarlos): This should be changed in the future to some API.
  //
  std::shared_ptr<openmapper::OpenMapper> openmapper_engine_;
};

}  // namespace openmapper

#endif  // INCLUDE_OPENMAPPER_RENDERER_H_
