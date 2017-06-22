// (c) 2017 OpenMapper

#ifndef WRAPPER_H_
#define WRAPPER_H_

#include <chrono>
#include <ctime>
#include <iostream>
#include <ratio>
#include <string>
#include <thread>
#include <vector>

#include <System.h>

namespace openmapper_wrapper {

class Wrapper {
 public:
  bool is_tracking;

  //
  // OpenCV Matrix containint the current image.
  //
  cv::Mat curr_image;

  //
  // OpenCV Matrix containing the current camera transformation from the origin
  // to the current pose.
  //
  cv::Mat curr_cam_transformation;

  //
  // Time stamp of the last image in seconds, measured as time since epoch.
  //
  double curr_frame_time_stamp;

  //
  // Input sensor
  //
  enum VideoSource { kCamera = 0, kFile = 1 };

  //
  // Main instance of the SLAM engine used.
  //
  ORB_SLAM2::System slam_engine;

  struct CameraPose {
    // TODO(gocarlos): define camera_pos and camera_rot.
    //
    //
    //
    std::vector<double> camera_pos;

    //
    //
    //
    std::vector<double> camera_rot;
  } cam_pose;

  struct SceneMap {
    std::size_t num_points = 0;

  } scene_map;

  //
  // Constructor
  //
  Wrapper(const std::vector<std::string>& flags);

  //
  // Destructor
  //
  virtual ~Wrapper();

  //
  // When called, starts the engine in order to track the camera.
  // @param flags is a vector of strings containing the flags,
  // those flags are the settings for tracking
  // flags[0] = path_to_vocabulary
  // flags[1] = path_to_settings (camera dependent, *.yaml file)
  // @param input_file is the string corresponding to the path to the video
  // file.
  //
  int StartSLAM(const VideoSource source, const std::string input_file);

  //
  // When called, stops the SLAM engine.
  //
  void StopSLAM();

  //
  // When called, returns the pose of the camera in the inertial frame.
  // @param pos is a vector with the absolute position x, y, z
  // @param rot is a vector with the absolute rotation as quaternion x, y, z, w
  //
  void GetPose(std::vector<double>& pos, std::vector<double>& rot);

  //
  // When called prints some information about the status of the current
  // process.
  //
  void DebugInfo();

  void GetCurrTimeSec(double& time);
};

}  // namespace openmapper_wrapper

#endif  // WRAPPER_H_
