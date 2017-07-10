// (c) 2017 OpenMapper

#include "openmapper/renderer.h"

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

}  // namespace openmapper
