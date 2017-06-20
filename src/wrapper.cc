// (c) 2017 OpenMapper

#include "../include/wrapper.h"

namespace openmapper_wrapper {

Wrapper::Wrapper(const std::vector<std::string>& flags) :
		slam_engine(flags[0], flags[1], ORB_SLAM2::System::MONOCULAR, true) {
}

Wrapper::~Wrapper() {
}

int Wrapper::StartSLAM(const VideoSource source, const std::string input_file) {
	cv::Mat image;
	cv::Mat cam_transformation;
	assert(input_file == "");

	time_t start_time_seconds = time(NULL);

	if (source == kCamera) {
		cv::VideoCapture cap(0);
		// Open the default camera.
		if (!cap.isOpened()) {
			// Check if we succeeded.
			std::cerr << "Camera input is broken!" << std::endl;
			return -1;
		}

		while (true) {
			// Get a new frame from camera.
			cap >> image;

			time_t seconds = time(NULL);
			double tframe = seconds;

			if (image.empty()) {
				// TODO(gocarlos): check here if the stream is finished or could not read.
				std::cerr << std::endl
						<< "Failed to another load image, eventually finished?"
						<< std::endl;
				return 1;
			}

			// Pass the image to the SLAM system.
			cam_transformation = slam_engine.TrackMonocular(image, tframe);
			std::cout << "Camera transformation: \n" << cam_transformation
					<< std::endl;
			double num_seconds = seconds - start_time_seconds;

		}
	}

	if (source == kFile) {
		cv::VideoCapture cap(input_file);
		if (!cap.isOpened()) {
			// Check if we succeeded.
			std::cerr << "Movie input is broken!" << std::endl;
			return -1;
		}
		while (true) {
			// Get a new frame from camera.
			cap >> image;

			time_t current_time_seconds = time(NULL);
			double tframe = current_time_seconds;

			if (image.empty()) {
				// TODO(gocarlos): check here if the stream is finished or could not read.
				std::cerr << std::endl
						<< "Failed to another load image, eventually finished?"
						<< std::endl;
				break;
			}

			// Pass the image to the SLAM system.
			cam_transformation = slam_engine.TrackMonocular(image, tframe);
			std::cout << "Camera transformation at time "
					<< current_time_seconds << "\n" << cam_transformation
					<< "\n" << cam_transformation.rows << std::endl << "\n"
					<< cam_transformation.cols << std::endl;
			camera_pos.resize(3);
//			camera_pos[0] = cam_transformation.at<double>(0,3);
//			camera_pos[1] = cam_transformation.at<double>(1,3);
//			camera_pos[2] = cam_transformation.at<double>(2,3);

			if (current_time_seconds - start_time_seconds > 30) {
				break;
			}

		}
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
