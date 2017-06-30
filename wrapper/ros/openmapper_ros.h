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
  void publishImage();

 private:
  // ROS nodehandle.
  ros::NodeHandle& nodeHandle_;

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
};

}  // namespace  openmapper_ros

#endif  // ROS_IMAGE_GRABBER_H_
