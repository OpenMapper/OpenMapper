// (c) 2017 OpenMapper

#ifndef ROS_IMAGE_GRABBER_H_
#define ROS_IMAGE_GRABBER_H_

#include <opencv2/core/core.hpp>

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

//#include "System.h"
#include "openmapper/wrapper.h"

namespace open_mapper_ros {

class WrapperROS {
 public:
  WrapperROS(int argc, char** argv, ros::NodeHandle& nodeHandle);

  void GrabImage(const sensor_msgs::ImageConstPtr& msg);
  void PublishPose();
  void PublishLandMarks();

 private:
  // ROS nodehandle.
  ros::NodeHandle& nodeHandle_;

  cv_bridge::CvImageConstPtr cv_ptr_;

  ros::Publisher marker_pub_;
  ros::Publisher position_pub_;

  std::string camera_frame = "/camera_frame";
  std::string world_frame= "/world";

  openmapper_wrapper::Wrapper wrapper_;
};

}  // namespace  open_mapper_ros

#endif  // ROS_IMAGE_GRABBER_H_
