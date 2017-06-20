// (c) 2017 OpenMapper

#include <iostream>
#include <vector>
#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>

#include "wrapper.h"

#include <System.h>

#include <opencv2/core/core.hpp>

#include <System.h>
#include <stdio.h>
#include <time.h>

int main(int argc, char** argv) {
  std::cout << "Hello World!" << std::endl;

  if (argc != 3) {
    cerr << endl
         << "Usage: ./mono_main path_to_vocabulary path_to_settings " << endl;
    return 1;
  }

  cv::VideoCapture cap(0);  // open the default camera
  if (!cap.isOpened()) {    // check if we succeeded
    return -1;
  }
  cout << "Could capture images " << endl << endl;

  cv::Mat edges;
  cv::Mat im;

  // Create SLAM system. It initializes all system threads and gets ready to
  // process frames.
  ORB_SLAM2::System SLAM(argv[1], argv[2], ORB_SLAM2::System::MONOCULAR, true);

  cout << endl << "-------" << endl;
  cout << "Start processing sequence ..." << endl;
  while (true) {
    cap >> im;  // get a new frame from camera

    time_t seconds = time(NULL);
    double tframe = seconds;

    if (im.empty()) {
      cerr << endl << "Failed to load image" << endl;
      return 1;
    }

    // Pass the image to the SLAM system
    SLAM.TrackMonocular(im, tframe);

    sleep(0.001);
  }

  // Stop all threads
  SLAM.Shutdown();

  // Save camera trajectory
  SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

  return 0;
}
