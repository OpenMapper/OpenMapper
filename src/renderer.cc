// (c) 2017 OpenMapper

#include "openmapper/renderer.h"

#include <GL/gl.h>
#include <opencv2/opencv.hpp>

namespace openmapper {

Renderer::Renderer() {}

void Renderer::displayImage(const cv::Mat& img) {}

void Renderer::displayFeatures(const std::vector<cv::Point3f>& all_map_points) {

}

void Renderer::projectPoints(
    const std::vector<cv::Point3f>& all_map_points,
    std::shared_ptr<std::vector<cv::Point2f>> projected_points) {
  cv::Mat pose;
  cv::Mat rot_vec;
  cv::Mat distCoeffs;
  cv::Mat cameraMatrix;
  cv::Mat trans_vec = pose.col(3).rowRange(0, 3);
  cv::Rodrigues(pose.colRange(0, 3).rowRange(0, 3), rot_vec);
  cv::projectPoints(all_map_points, rot_vec, trans_vec, cameraMatrix,
                    distCoeffs, *projected_points);
}

void Renderer::drawMapPoints() {
  CHECK_NOTNULL(openmapper_engine_.get());
  const std::vector<cv::Point3f>& vpMPs =
      openmapper_engine_->map_->getFeaturesPosition();

  const float point_size = 2.0f;
  if (vpMPs.size() > 0) {
    glPointSize(point_size);
    glBegin(GL_POINTS);
    glColor3f(1.0, 0.0, 0.0);

    for (std::size_t i = 0u; i < vpMPs.size(); ++i) {
      glVertex3f(vpMPs[i].x, vpMPs[i].y, vpMPs[i].z);
    }
    glEnd();
  }
}

}  // namespace openmapper
