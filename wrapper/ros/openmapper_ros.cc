// (c) 2017 OpenMapper

#include "openmapper_ros.h"
#include <csignal>

namespace openmapper_ros {

WrapperROS::WrapperROS(int argc, char** argv, ros::NodeHandle& nodeHandle)
    : nodeHandle_(nodeHandle),
      flags_{argv[1], argv[2]},
      openmapper_engine_(flags_),
      renderer_(new openmapper::Renderer) {
  initialize(argv);

  // ROS publisher for the position of the camera.
  marker_pub_ = nodeHandle_.advertise<visualization_msgs::MarkerArray>(
      "/visualization_marker", 1);
  position_pub_ =
      nodeHandle_.advertise<geometry_msgs::PoseStamped>("/camera_pose", 1);
  image_pub_ =
      nodeHandle_.advertise<sensor_msgs::Image>("/camera/image_raw", 1);
  signal(SIGINT, inthand);

  trackCamera();
}

bool WrapperROS::stop = false;

void WrapperROS::inthand(int signum) {
  LOG(WARNING) << "Interrupt signal (" << signum << ") received.";
  stop = true;
}

void WrapperROS::grabROSImage(const sensor_msgs::ImageConstPtr& msg) {
  // Copy the ROS image message to cv::Mat.
  try {
    cv_ptr_ = cv_bridge::toCvShare(msg);
    input_source_->setCurrentImage(cv_ptr_->image);
    input_source_->setCurrentImageTimeSec(cv_ptr_->header.stamp.toSec());
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }
  // TODO(gocarlos): test this.
}

void WrapperROS::initialize(char** argv) {
  CHECK_NOTNULL(argv);
  // Choose the input topic.
  const std::string input = argv[3];
  const std::string settings = argv[4];

  input_source_.reset(new openmapper::InputSource());

  // string::compare == 0 returns true if both strings are equal .
  if (input.compare("opencv_live") == 0) {
    // Use the openCV camera.
    camera_stream_live_input_ = settings;
    LOG(INFO) << "Source is set to live video from camera device "
              << camera_stream_live_input_;
    input_source_->setInput(openmapper::InputSource::kCamera,
                            camera_stream_live_input_);
  } else if (input.compare("opencv_movie") == 0) {
    // Read from file.
    camera_stream_movie_path_ = settings;
    LOG(INFO) << "Source is set to video file from path "
              << camera_stream_movie_path_;
    input_source_->setInput(openmapper::InputSource::kFile,
                            camera_stream_movie_path_);
  } else if (input.compare("ros_topic") == 0) {
    // Use ROS topic.
    camera_stream_ros_topic_ = settings;
    LOG(INFO) << "Source is set to life video from ROS topic "
              << camera_stream_ros_topic_;

    // ROS subscriber to the images in this topic.
    ros::Subscriber sub = nodeHandle_.subscribe(
        camera_stream_ros_topic_, 1, &WrapperROS::grabROSImage, this);
    input_source_->setInput(openmapper::InputSource::kImage,
                            camera_stream_ros_topic_);
  } else {
    LOG(FATAL) << "ERROR: no image source have been chosen!";
  }
}

void WrapperROS::publishPose() {
  std::shared_ptr<std::vector<double>> pos(new std::vector<double>);
  std::shared_ptr<std::vector<double>> rot(new std::vector<double>);

  // Get pose of the camera in the fixed coordinate system.
  openmapper_engine_.getPose(pos, rot);

  // Publish the pose of the camera.
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = fixed_frame_;
  pose.header.stamp = ros::Time::now();
  pose.pose.position.x = (*pos)[0];
  pose.pose.position.y = (*pos)[1];
  pose.pose.position.z = (*pos)[2];

  pose.pose.orientation.x = (*rot)[0];
  pose.pose.orientation.y = (*rot)[1];
  pose.pose.orientation.z = (*rot)[2];
  pose.pose.orientation.w = (*rot)[3];

  position_pub_.publish(pose);

  // Publish a ROS transform.
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin(tf::Vector3((*pos)[0], (*pos)[1], (*pos)[2]));
  tf::Quaternion q;
  q.setValue((*rot)[0], (*rot)[1], (*rot)[2], (*rot)[3]);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(),
                                        fixed_frame_, camera_frame_));
}

void WrapperROS::publishLandMarks() {
  if (openmapper_engine_.getSlamEngine()->GetTrackingState() == 2) {
    // The tracking state of the low level engine is 2 if tracking is okay.

    const std::vector<cv::Point3f>& all_map_points =
        openmapper_engine_.map_->getFeaturesPosition();

    LOG(INFO) << "Number of tracked map points: " << all_map_points.size();

    if (all_map_points.size() > 0) {
      visualization_msgs::MarkerArray markers;
      for (size_t i = 0u; i < all_map_points.size(); ++i) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = fixed_frame_;
        marker.header.stamp = ros::Time::now();
        marker.ns = "map_features";
        marker.id = i;

        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;

        marker.pose.position.x = all_map_points[i].x;
        marker.pose.position.y = all_map_points[i].y;
        marker.pose.position.z = all_map_points[i].z;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        marker.scale.x = 0.01;
        marker.scale.y = 0.01;
        marker.scale.z = 0.01;

        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0f;

        ros::Duration marker_durability;
        marker_durability.sec = 1.0 / input_source_->fps_;
        marker.lifetime = marker_durability;
        markers.markers.push_back(marker);
      }
      marker_pub_.publish(markers);
    }
  }
}

void WrapperROS::publishImage(const cv::Mat& img) {
  cv_bridge::CvImage img_bridge;
  sensor_msgs::Image img_msg;
  std_msgs::Header header;

  header.stamp = ros::Time::now();
  img_bridge =
      cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, img);
  img_bridge.toImageMsg(img_msg);
  image_pub_.publish(img_msg);
}

void WrapperROS::trackCamera() {
  CHECK_NOTNULL(input_source_.get());
  CHECK(input_source_->isInputModeSet());

  double curr_time = 0.0;
  openmapper::Common::getCurrTimeSec(curr_time);
  double start_time = 0.0;
  openmapper::Common::getCurrTimeSec(start_time);
  while (true) {
    input_source_->grabImage();
    cv::Mat img = input_source_->getCurrentImage();

    bool tracking = openmapper_engine_.trackImage(
        img, input_source_->getCurrentImageTimeSec());
    if (!tracking || stop) {
      break;
    }

    sleep(1.0 / input_source_->fps_);
    openmapper::Common::getCurrTimeSec(curr_time);

    if (curr_time - start_time > kPublishCycleTime_) {
      openmapper::Common::getCurrTimeSec(start_time);

      publishImage(img);
      publishLandMarks();
      publishPose();
    }
    ros::spinOnce();
  }
}

}  // namespace  openmapper_ros
