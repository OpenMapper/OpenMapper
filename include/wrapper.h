// (c) 2017 OpenMapper

#ifndef WRAPPER_H_
#define WRAPPER_H_

#include <vector>
#include <string>

#include <System.h>

namespace openmapper_wrapper {

class Wrapper {
public:

	// Input sensor
	enum VideoSource {
		kCamera = 0, kFile = 1
	};

	ORB_SLAM2::System slam_engine;

	//
	// Constructor
	//
	Wrapper(const std::vector<std::string> &flags);

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
	//
	int StartSLAM(const VideoSource source);

	//
	// When called, stops the SLAM engine.
	//
	void StopSLAM();

	//
	// When called, returns the pose of the camera in the inertial frame.
	// @param pos is a vector with the absolute position x, y, z
	// @param rot is a vector with the absolute rotation as quaternion x, y, z, w
	//
	void GetPose(std::vector<double> &pos, std::vector<double> &rot);

};

} // namespace openmapper_wrapper

#endif // WRAPPER_H_
