// (c) 2017 OpenMapper

#include "openmapper_ros.h"

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_alsologtostderr = 1;
  FLAGS_colorlogtostderr = 1;
  google::InstallFailureSignalHandler();

  if (argc < 5) {
    LOG(FATAL) << "Example usage: rosrun open_mapper_ros open_mapper_ros "
                  "path_to_vocabulary path_to_settings opencv_live 0";
  } else {
    LOG(INFO) << "Going to print out the arguments: ";
    for (std::size_t i = 0u; i < argc; ++i) {
      std::cout << "-->" << i << "--" << argv[i] << "<--" << std::endl;
    }
  }

  ros::init(argc, argv, "open_mapper_ros");
  ros::NodeHandle nodeHandle("~");
  openmapper_ros::WrapperROS wrapper_ros(argc, argv, nodeHandle);
  return 0;
}
