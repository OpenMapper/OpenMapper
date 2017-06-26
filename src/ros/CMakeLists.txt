cmake_minimum_required(VERSION 3.1)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

## System dependencies are found with CMake's conventions
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED)
find_package(Threads)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES opencv
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs cv_bridge opencv image_transport pthread
#   DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 # include
 	"/usr/local/include"
  ${CMAKE_CURRENT_LIST_DIR}/thirdparty/slam_engine/ORB_SLAM2/include
  ${CMAKE_CURRENT_LIST_DIR}/thirdparty/slam_engine/ORB_SLAM2/
  ${CMAKE_CURRENT_LIST_DIR}/thirdparty/slam_engine/ORB_SLAM2/Thirdparty/DBoW2/DBoW2
  ${CMAKE_CURRENT_LIST_DIR}/thirdparty/slam_engine/ORB_SLAM2/Thirdparty/g2o/g2o/core
  ${CMAKE_CURRENT_LIST_DIR}/thirdparty/slam_engine/ORB_SLAM2/Thirdparty/g2o/g2o/solvers
  ${CMAKE_CURRENT_LIST_DIR}/thirdparty/slam_engine/ORB_SLAM2/Thirdparty/g2o/g2o/stuff
  ${CMAKE_CURRENT_LIST_DIR}/thirdparty/slam_engine/ORB_SLAM2/Thirdparty/g2o/g2o/types
  ${EIGEN3_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# Declare a C++ library
add_library(ASDFASDF)

target_link_libraries(ASDFASDF
  debug ${OM_WRAPPER_LIB} optimized ${OM_WRAPPER_LIB}
)

add_executable(${PROJECT_NAME}_node
	src/ros/main_node.cc
)

add_dependencies(${PROJECT_NAME}_node
	${${PROJECT_NAME}_EXPORTED_TARGETS}
	${catkin_EXPORTED_TARGETS}
  ${OM_WRAPPER_LIB}
  ${ASDFASDF}
)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
  Threads::Threads
 	debug ${OpenCV_LIBS} optimized ${OpenCV_LIBS}
  debug ${OM_WRAPPER_LIB} optimized ${OM_WRAPPER_LIB}
  debug ${catkin_LIBRARIES} optimized ${catkin_LIBRARIES} # ROS stuff
)

message(STATUS "    ")
message(STATUS "    ")
message(STATUS "    ")

message(STATUS "OpenCV library status:")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBRARIES}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")
message(STATUS "    ")
message(STATUS "    ")
message(STATUS "    ")

message(STATUS "OpenCV library status:")
message(STATUS "    {PROJECT_NAME}_EXPORTED_TARGETS} : ${${PROJECT_NAME}_EXPORTED_TARGETS}")
message(STATUS "    catkin_EXPORTED_TARGETS: ${catkin_EXPORTED_TARGETS}")
message(STATUS "    OM_WRAPPER_LIB: ${OM_WRAPPER_LIB}")

message(STATUS "    ")
message(STATUS "    ")
message(STATUS "    ")
