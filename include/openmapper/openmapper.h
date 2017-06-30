// (c) 2017 OpenMapper

#ifndef INCLUDE_OPENMAPPER_OPENMAPPER_H_
#define INCLUDE_OPENMAPPER_OPENMAPPER_H_

#include <ctime>
#include <iomanip>
#include <iostream>
#include <ratio>
#include <string>
#include <thread>
#include <vector>

// ORB_SLAM2
#include <System.h>

// OpenMapper
#include "openmapper/common.h"
#include "openmapper/input_source.h"
#include "openmapper/pose.h"

namespace openmapper {

class OpenMapper {
 public:
  std::string path_to_vocabulary_;
  std::string path_to_settings_;

  InputSource input_source_;
  Pose pose_;

  //
  // Constructor
  // @param flags is a vector of strings containing the flags,
  // those flags are the settings for tracking
  // flags[0] = path_to_vocabulary
  // flags[1] = path_to_settings (camera dependent, *.yaml file)
  //
  OpenMapper(const std::vector<std::string>& flags);
  OpenMapper(int argc, char** argv);
  void initialize();
  //
  // Destructor
  //
  virtual ~OpenMapper();

  //
  // When called, starts the engine in order to track the camera.
  // @param input_file is the string corresponding to the path to the video
  // file.
  // @param source is the input source for the video, web camera or video file.
  //
  bool trackImage();

  //
  // When called, stops the SLAM engine.
  //
  void stopSLAM();

  //
  // When called, returns the pose of the camera in the inertial frame.
  // @param pos is a vector with the absolute position x, y, z
  // @param rot is a vector with the absolute rotation as quaternion x, y, z, w
  //
  void getPose(std::shared_ptr<std::vector<double>> pos,
               std::shared_ptr<std::vector<double>> rot);

 private:
  //
  // Main instance of the SLAM engine used.
  //
  ORB_SLAM2::System slam_engine;
};

}  // namespace openmapper_wrapper

#endif  // INCLUDE_OPENMAPPER_OPENMAPPER_H_
