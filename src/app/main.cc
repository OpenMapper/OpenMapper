// (c) 2017 OpenMapper

//#include "image_grabber.h"

#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>

#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>

#include <opencv2/core/core.hpp>

#include "System.h"

class ImageGrabber {
 public:
  ImageGrabber(ORB_SLAM2::System* pSLAM) : mpSLAM(pSLAM) {}

  void GrabImage(const sensor_msgs::ImageConstPtr& msg);

  ORB_SLAM2::System* mpSLAM;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "Mono");
  ros::start();

  if (argc != 3) {
    cerr << endl
         << "Usage: rosrun ORB_SLAM2 Mono path_to_vocabulary path_to_settings"
         << endl;
    ros::shutdown();
    return 1;
  }

  // Create SLAM system. It initializes all system threads and gets ready to
  // process frames.
  ORB_SLAM2::System SLAM(argv[1], argv[2], ORB_SLAM2::System::MONOCULAR, true);

  ImageGrabber igb(&SLAM);

  ros::NodeHandle nodeHandler;
  ros::Subscriber sub = nodeHandler.subscribe("/camera/image_raw", 1,
                                              &ImageGrabber::GrabImage, &igb);

  ros::spin();

  // Stop all threads
  SLAM.Shutdown();

  ros::shutdown();

  return 0;
}

void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msg) {
      // Copy the ros image message to cv::Mat.
      cv_bridge::CvImageConstPtr cv_ptr;
      try
      {
          cv_ptr = cv_bridge::toCvShare(msg);
      }
      catch (cv_bridge::Exception& e)
      {
          ROS_ERROR("cv_bridge exception: %s", e.what());
          return;
      }

      mpSLAM->TrackMonocular(cv_ptr->image,cv_ptr->header.stamp.toSec());
}
//
//// (c) 2017 OpenMapper
//
//#include <algorithm>
//#include <chrono>
//#include <fstream>
//#include <iostream>
//#include <vector>
//
//#include "wrapper.h"
//
//#include <System.h>
//
//#include <opencv2/core/core.hpp>
//
//#include <System.h>
//#include <stdio.h>
//#include <time.h>
//
//int main(int argc, char** argv) {
//  std::cout << "Hello World!" << std::endl;
//
//  std::string path_to_vocabulary =
//      "../thirdparty/slam_engine/ORB_SLAM2/Vocabulary/ORBvoc.txt";
//  std::string path_to_settings =
//      "../thirdparty/slam_engine/ORB_SLAM2/Vocabulary/webcam.yaml";
//
//  if (argc != 3) {
//    cerr << endl
//         << "Usage: ./mono_main path_to_vocabulary path_to_settings " << endl;
//    path_to_vocabulary =
//        "../thirdparty/slam_engine/ORB_SLAM2/Vocabulary/ORBvoc.txt";
//    path_to_settings =
//        "../thirdparty/slam_engine/ORB_SLAM2/Vocabulary/webcam.yaml";
//  } else {
//    path_to_vocabulary = argv[1];
//    path_to_settings = argv[2];
//  }
//
//  cv::VideoCapture cap(0);  // open the default camera
//  if (!cap.isOpened()) {    // check if we succeeded
//    return -1;
//  }
//  cout << "Could capture images " << endl << endl;
//
//  cv::Mat edges;
//  cv::Mat im;
//
//  // Create SLAM system. It initializes all system threads and gets ready to
//  // process frames.
//  ORB_SLAM2::System SLAM(path_to_vocabulary, path_to_settings,
//                         ORB_SLAM2::System::MONOCULAR, false);
//
//  cout << endl << "-------" << endl;
//  cout << "Start processing sequence ..." << endl;
//  while (true) {
//    cap >> im;  // get a new frame from camera
//
//    time_t seconds = time(NULL);
//    double tframe = seconds;
//
//    if (im.empty()) {
//      cerr << endl << "Failed to load image" << endl;
//      return 1;
//    }
//
//    // Pass the image to the SLAM system
//    SLAM.TrackMonocular(im, tframe);
//
//    sleep(0.001);
//  }
//
//  // Stop all threads
//  SLAM.Shutdown();
//
//  // Save camera trajectory
//  SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
//
//  return 0;
//}
