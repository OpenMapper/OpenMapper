// (c) 2017 OpenMapper

#include "wrapper_ros.h"

namespace open_mapper_ros {

WrapperROS::WrapperROS(int argc, char** argv, ros::NodeHandle& nodeHandle)
    : nodeHandle_(nodeHandle), wrapper_(argc, argv) {
  // ROS subscriber to the images in this topic.
  ros::Subscriber sub = nodeHandle_.subscribe("/camera/image_raw", 1,
                                              &WrapperROS::GrabImage, this);

  // ROS publisher for the position of the camera.
  marker_pub_ = nodeHandle_.advertise<visualization_msgs::Marker>(
      "visualization_marker", 1);
  position_pub_ =
      nodeHandle_.advertise<geometry_msgs::PoseStamped>("camera_pose", 1);
  if (argc != 3) {
    cerr << endl
         << "Usage: rosrun OpenMapper ros_node path_to_vocabulary "
            "path_to_settings"
         << endl;
    ros::shutdown();
    exit(1);
  }
}

void WrapperROS::GrabImage(const sensor_msgs::ImageConstPtr& msg) {
  // Copy the ROS image message to cv::Mat.
  try {
    cv_ptr_ = cv_bridge::toCvShare(msg);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }
  *(wrapper_.input_source.current_image) = (cv_ptr_->image);
}

void WrapperROS::PublishPose() {
  std::shared_ptr<std::vector<double>> pos;
  std::shared_ptr<std::vector<double>> rot;
  wrapper_.GetPose(pos, rot);

  visualization_msgs::Marker marker;
  marker.header.frame_id = world_frame;
  marker.header.stamp = ros::Time::now();
  marker.ns = "camera";
  marker.id = 0u;

  marker.type = visualization_msgs::Marker::ARROW;
  marker.action = visualization_msgs::Marker::ADD;

  marker.pose.position.x = (*pos)[0];
  marker.pose.position.y = (*pos)[1];
  marker.pose.position.z = (*pos)[2];
  marker.pose.orientation.x = (*rot)[0];
  marker.pose.orientation.y = (*rot)[1];
  marker.pose.orientation.z = (*rot)[2];
  marker.pose.orientation.w = (*rot)[3];

  marker.scale.x = 0.2;
  marker.scale.y = 0.2;
  marker.scale.z = 0.2;

  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;

  marker.lifetime = ros::Duration();
  marker_pub_.publish(marker);

  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = world_frame;
  pose.header.stamp = ros::Time::now();
  pose.pose.position.x = (*pos)[0];
  pose.pose.position.y = (*pos)[1];
  pose.pose.position.z = (*pos)[2];

  pose.pose.orientation.x = (*rot)[0];
  pose.pose.orientation.y = (*rot)[1];
  pose.pose.orientation.z = (*rot)[2];
  pose.pose.orientation.w = (*rot)[3];

  position_pub_.publish(pose);
}

void WrapperROS::PublishLandMarks() {}

}  // namespace  open_mapper_ros
