// (c) 2017 OpenMapper

#ifndef INCLUDE_OPENMAPPER_WRAPPER_H_
#define INCLUDE_OPENMAPPER_WRAPPER_H_

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

namespace openmapper_wrapper {

class Wrapper {
 public:
  std::string path_to_vocabulary;
  std::string path_to_settings;

  InputSource input_source;

  // TODO(gocarlos): get this state from  FrameDrawer::eTrackingState mState;
  bool has_tracked;

  //
  // OpenCV Matrix containing the current image.
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
  enum VideoSource { kCamera = 0, kFile = 1, kImage = 3 };

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
  // @param flags is a vector of strings containing the flags,
  // those flags are the settings for tracking
  // flags[0] = path_to_vocabulary
  // flags[1] = path_to_settings (camera dependent, *.yaml file)
  //
  Wrapper(const std::vector<std::string>& flags);
  Wrapper(int argc, char** argv);
  void Initialize();
  //
  // Destructor
  //
  virtual ~Wrapper();

  //
  // When called, starts the engine in order to track the camera.
  // @param input_file is the string corresponding to the path to the video
  // file.
  // @param source is the input source for the video, web camera or video file.
  //
  int StartSLAM(const VideoSource source, const std::string input_file);

  //  mpSLAM->TrackMonocular(cv_ptr->image, cv_ptr->header.stamp.toSec());

  //
  // When called, stops the SLAM engine.
  //
  void StopSLAM();

  //
  // When called, returns the pose of the camera in the inertial frame.
  // @param pos is a vector with the absolute position x, y, z
  // @param rot is a vector with the absolute rotation as quaternion x, y, z, w
  //
  void GetPose(std::shared_ptr<std::vector<double>> pos,
               std::shared_ptr<std::vector<double>> rot);

  //
  // When called prints some information about the status of the current
  // process.
  //
  void DebugInfo();

 private:
  //
  // Main instance of the SLAM engine used.
  //
  ORB_SLAM2::System slam_engine;
};

}  // namespace openmapper_wrapper

#endif  // INCLUDE_OPENMAPPER_WRAPPER_H_
