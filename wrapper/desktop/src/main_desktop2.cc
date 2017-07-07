//// (c) 2017 OpenMapper
//
//#include <algorithm>
//#include <chrono>
//#include <fstream>
//#include <iostream>
//#include <vector>
//#include <stdio.h>
//#include <time.h>
//
//#include <glad/glad.h>
//#include <GLFW\glfw3.h>
//
//#include <System.h>
//
//#include <opencv2/core/core.hpp>
//
//
//#include "openmapper/openmapper.h"
//#include "config.h"
//
//int main(int argc, char** argv) {
//  std::cout << "Hello World!" << std::endl;
//
//
//
////  if (argc != 3) {
////    std::cerr << std::endl
////         << "Usage: ./mono_main path_to_vocabulary path_to_settings " << std::endl;
////    path_to_vocabulary =
////        "../thirdparty/slam_engine/ORB_SLAM2/Vocabulary/ORBvoc.txt";
////    path_to_settings =
////        "../thirdparty/slam_engine/ORB_SLAM2/Vocabulary/webcam.yaml";
////  } else {
////    path_to_vocabulary = argv[1];
////    path_to_settings = argv[2];
////  }
////
////  cv::VideoCapture cap(0);  // open the default camera
////  if (!cap.isOpened()) {    // check if we succeeded
////    return -1;
////  }
////  std::cout << "Could capture images " << std::endl << std::endl;
////
////  cv::Mat im;
////
////  // Create SLAM system. It initializes all system threads and gets ready to
////  // process frames.
////  ORB_SLAM2::System SLAM(path_to_vocabulary, path_to_settings,
////                         ORB_SLAM2::System::MONOCULAR, false);
////
////  std::cout << std::endl << "-------" << std::endl;
////  std::cout << "Start processing sequence ..." << std::endl;
////  while (true) {
////    cap >> im;  // get a new frame from camera
////
////    time_t seconds = time(NULL);
////    double tframe = seconds;
////
////    if (im.empty()) {
////    	std::cerr << std::endl << "Failed to load image" << std::endl;
////      return 1;
////    }
////
////    // Pass the image to the SLAM system
////    SLAM.TrackMonocular(im, tframe);
////
////    sleep(0.001);
////  }
////
////  // Stop all threads
////  SLAM.Shutdown();
////
////  // Save camera trajectory
////  SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
//
//  return 0;
//}
