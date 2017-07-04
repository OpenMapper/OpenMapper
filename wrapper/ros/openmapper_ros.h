// (c) 2017 OpenMapper

#ifndef ROS_IMAGE_GRABBER_H_
#define ROS_IMAGE_GRABBER_H_

#include <string>
#include <thread>

#include <opencv2/core/core.hpp>

// ROS
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// OpenMapper
#include "openmapper/input_source.h"
#include "openmapper/openmapper.h"

namespace openmapper_ros {

class WrapperROS {
 public:
  WrapperROS(int argc, char** argv, ros::NodeHandle& nodeHandle);

  void initialize(char** argv);

  void grabROSImage(const sensor_msgs::ImageConstPtr& msg);
  void trackCamera();
  void publishPose();
  void publishLandMarks();
  void publishImage(const cv::Mat& img);
  static void inthand(int signum);

 private:
  // ROS nodehandle.
  ros::NodeHandle& nodeHandle_;
  std::shared_ptr<openmapper::InputSource> input_source_;

  cv_bridge::CvImageConstPtr cv_ptr_;
  std::vector<std::string> flags_;

  ros::Publisher marker_pub_;
  ros::Publisher position_pub_;
  ros::Publisher image_pub_;

  std::string camera_stream_ros_topic_;
  std::string camera_stream_live_input_;
  std::string camera_stream_movie_path_;

  std::string camera_frame = "/camera_frame";
  std::string world_frame = "/world";

  openmapper::OpenMapper openmapper_engine_;
  const std::size_t kPublishCycleTime_ = 5u;
  static bool stop;
};

}  // namespace  openmapper_ros

#endif  // ROS_IMAGE_GRABBER_H_
