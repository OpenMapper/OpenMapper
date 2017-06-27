//// (c) 2017 OpenMapper
//
//#include "image_grabber.h"
//
//void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msg) {
//  // Copy the ros image message to cv::Mat.
//  cv_bridge::CvImageConstPtr cv_ptr;
//  try {
//    cv_ptr = cv_bridge::toCvShare(msg);
//  } catch (cv_bridge::Exception& e) {
//    ROS_ERROR("cv_bridge exception: %s", e.what());
//    return;
//  }
//
//  mpSLAM->TrackMonocular(cv_ptr->image, cv_ptr->header.stamp.toSec());
//}
