// (c) 2017 OpenMapper

#include "wrapper_ros.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "openmapper_wrapper_ros");
  ros::NodeHandle nodeHandle("~");
  open_mapper_ros::WrapperROS wrapper_ros(argc, argv,nodeHandle);
  ros::spin();
  return 0;
}


//#include <algorithm>
//#include <chrono>
//#include <fstream>
//#include <iostream>
//
//int main(int argc, char** argv) {
//  ros::init(argc, argv, "Mono");
//  ros::start();
//
//  if (argc != 3) {
//    cerr << endl
//         << "Usage: rosrun ORB_SLAM2 Mono path_to_vocabulary path_to_settings"
//         << endl;
//    ros::shutdown();
//    return 1;
//  }
//
//  // Create SLAM system. It initializes all system threads and gets ready to
//  // process frames.
//  ORB_SLAM2::System SLAM(argv[1], argv[2], ORB_SLAM2::System::MONOCULAR, true);
//
//  ImageGrabber igb(&SLAM);
//
//  ros::NodeHandle nodeHandler;
//  ros::Subscriber sub = nodeHandler.subscribe("/camera/image_raw", 1,
//                                              &ImageGrabber::GrabImage, &igb);
//
//  ros::spin();
//
//  // Stop all threads
//  SLAM.Shutdown();
//
//  ros::shutdown();
//
//  return 0;
//}
