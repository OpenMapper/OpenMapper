//============================================================================
// Name        : wrapper.h
// Author      : Carlos Gomes
// Copyright   : Copyright OpenMapper
// Description : Main class of the wrapper.
//============================================================================


#ifndef WRAPPER_H_
#define WRAPPER_H_

#include <vector>
#include <string>

namespace openmapper_wrapper {

class Wrapper {
public:
	//
	// Constructor
	//
	Wrapper();

	//
	// Destructor
	//
	virtual ~Wrapper();

	//
	// When called, starts the engine in order to track the camera.
	// @param flags is a vector of strings containing the flags,
	// those flags are the settings for tracking
	// TODO(gocarlos) specify the correct flags, pending as engine is not integrated yet.
	//
	void StartSLAM(const std::vector<std::string> &flags);

	//
	// When called, returns the pose of the camera in the inertial frame.
	// @param pos is a vector with the absolute position x, y, z
	// @param rot is a vector with the absolute rotation as quaternion x, y, z, w
	//
	void GetPose(std::vector<double> &pos, std::vector<double> &rot);

};

} // namespace openmapper_wrapper

#endif // WRAPPER_H_
