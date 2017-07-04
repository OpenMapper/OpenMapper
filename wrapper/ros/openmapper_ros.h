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
  // Constructor
  WrapperROS(int argc, char** argv, ros::NodeHandle& nodeHandle);

  // Is colled after the constructor: it sets the input source depending on the
  // input parameters.
  void initialize(char** argv);

  // ROS callback to get the latest images from a ROS topic.
  void grabROSImage(const sensor_msgs::ImageConstPtr& msg);

  // Main Loop inside this method.
  void trackCamera();

  // Publishes the pose of the camera into a ROS topic when called.
  void publishPose();

  // Publishes the map feature points into a ROS topic when called.
  void publishLandMarks();

  // Publishes the image of the camera into a ROS topic when called.
  void publishImage(const cv::Mat& img);

  // This method is called when SIGINT is received.
  static void inthand(int signum);

 private:
  // ROS nodehandle.
  ros::NodeHandle& nodeHandle_;

  //
  // The input source manages the input images. It gets the images over opencv
  // from a camera or movie.
  //
  std::shared_ptr<openmapper::InputSource> input_source_;

  //
  // ROS CV bridge converts ROS images into CV images. This is needed when
  // subscribing to images via topic and passing them to the low level engine as
  // opencv Mat.
  //
  cv_bridge::CvImageConstPtr cv_ptr_;

  //
  // Flags passed to the low level engine. Those are the Bag of words and camera
  // settings currently.
  //
  std::vector<std::string> flags_;

  //
  // ROS Publishers
  //
  ros::Publisher marker_pub_;
  ros::Publisher position_pub_;
  ros::Publisher image_pub_;

  //
  // Name of the input data stream.
  //
  std::string camera_stream_ros_topic_;
  std::string camera_stream_live_input_;
  std::string camera_stream_movie_path_;

  // ROS frames.
  std::string camera_frame = "/camera_frame";
  std::string world_frame = "/world";

  //  Instance of the openmapper engine.
  openmapper::OpenMapper openmapper_engine_;

  // The images, poses & landmarks are published to tha ROS topic every couple
  // of seconds.
  const std::size_t kPublishCycleTime_ = 5u;

  // This variable is used to get out of the main loop if SIGINT is received.
  static bool stop;
};

}  // namespace  openmapper_ros

#endif  // ROS_IMAGE_GRABBER_H_
