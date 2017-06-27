// (c) 2017 OpenMapper

#include "wrapper_ros.h"

int main(int argc, char** argv) {
  if (argc <= 5) {
    std::cerr << endl
              << "Usage: rosrun OpenMapper ros_node path_to_vocabulary "
                 "path_to_settings"
              << endl;
    exit(1);
  }
  ros::init(argc, argv, "openmapper_wrapper_ros");
  ros::NodeHandle nodeHandle("~");
  open_mapper_ros::WrapperROS wrapper_ros(argc, argv, nodeHandle);
  ros::spin();
  return 0;
}
