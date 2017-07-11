// (c) 2017 OpenMapper

#include "openmapper/renderer.h"

#ifdef __APPLE__
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#endif

#include <opencv2/opencv.hpp>

namespace openmapper {

Renderer::Renderer() {}

void Renderer::displayImage(const cv::Mat &img) {}

void Renderer::displayFeatures(const std::vector<cv::Point3f> &all_map_points) {

}

void Renderer::projectPoints(
    const std::vector<cv::Point3f> &all_map_points,
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
  const std::vector<cv::Point3f> &vpMPs =
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

void Renderer::drawCurrentImage() {
  CHECK_NOTNULL(openmapper_engine_.get());

  cv::Mat img2 = openmapper_engine_->cur_img_w_features_;
  GLuint imageTexture;

  glEnable(GL_TEXTURE_2D);

  // Create Texture
  glGenTextures(1, &imageTexture);
  // 2d texture (x and y size)
  glBindTexture(GL_TEXTURE_2D, imageTexture);
  // scale linearly when image bigger than texture
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  // scale linearly when image smalled than texture
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);

  // 2D texture, level of detail 0 (normal), 3 components (red, green, blue), x
  // size from image, y size from image, border 0 (normal), rgb color data,
  // unsigned byte data, and finally the data itself.
  glTexImage2D(GL_TEXTURE_2D, 0, 3, img2.cols, img2.rows, 0, GL_BGR,
               GL_UNSIGNED_BYTE, img2.data);

  // choose the texture to use.
  glBindTexture(GL_TEXTURE_2D, imageTexture);

  // Set global color to white, otherwise this color will be (somehow) added to
  // the texture
  glColor3f(1, 1, 1);

  glBegin(GL_QUADS);

  const float x = 1.0;
  const float y = 1.0;

  glTexCoord2f(0.0f, 0.0f);
  glVertex3f(-x, -y, 0.0f);
  glTexCoord2f(0.0f, 1.0f);
  glVertex3f(x, -y, 0.0f);
  glTexCoord2f(1.0f, 1.0f);
  glVertex3f(x, y, 0.0f);
  glTexCoord2f(1.0f, 0.0f);
  glVertex3f(-x, y, 0.0f);

  glEnd();
}

}  // namespace openmapper
