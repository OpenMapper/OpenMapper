// (c) 2017 OpenMapper

#include "../include/wrapper.h"

namespace openmapper_wrapper {

Wrapper::Wrapper(const std::vector<std::string>& flags) :
		slam_engine(flags[0], flags[1], ORB_SLAM2::System::MONOCULAR, false) {

	std::cout << "Flags: " <<"\n"<< flags[0] << "\n" << flags[1] << std::endl;

}

Wrapper::~Wrapper() {
}

int Wrapper::StartSLAM(const VideoSource source, const std::string input_file) {
	cv::Mat image;
	cv::Mat cam_transformation;
	cv::VideoCapture cap;
	assert(input_file == "");

	time_t start_time_seconds = time(NULL);

	if (source == kCamera) {
		cap.open(0);
	} else if (source == kFile) {
		cap.open(input_file);

	}

	// Open the default camera.
	if (!cap.isOpened()) {
		// Check if we succeeded.
		std::cerr << "Camera input is broken!" << std::endl;
		return -1;
	}

	while (true) {
		// Get a new frame from camera.
		cap >> image;

		auto now = std::chrono::system_clock::now();
		auto ms =
				std::chrono::time_point_cast<std::chrono::milliseconds>(now).time_since_epoch().count();

		double tframe = ms * 1000;

		if (image.empty()) {
			// TODO(gocarlos): check here if the stream is finished or could not read.
			std::cout << "Failed to load another image, eventually finished..."
					<< std::endl;
			return 0;
		}

		// Pass the image to the SLAM system.
		cam_transformation = slam_engine.TrackMonocular(image, tframe);

		std::cout << "Camera transformation at time " << std::setprecision(20)
				<< tframe << "\n" << cam_transformation.rows << " "
				<< cam_transformation.cols << "\n" << cam_transformation
				<< std::endl;
		camera_pos.resize(3);

//			camera_pos[0] = cam_transformation.at<double>(0,3);
//			camera_pos[1] = cam_transformation.at<double>(1,3);
//			camera_pos[2] = cam_transformation.at<double>(2,3);
	}

	return 0;
}

void Wrapper::StopSLAM() {
	// Stop all threads
	slam_engine.Shutdown();
}

void Wrapper::GetPose(std::vector<double>& pos, std::vector<double>& rot) {
	pos.resize(3);
	rot.resize(4);
	pos = {0.0, 0.0, 0.0};
	rot = {0.0, 0.0, 0.0, 1.0};
}
}
// namespace openmapper_wrapper
