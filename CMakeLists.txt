cmake_minimum_required (VERSION 2.8)
project (openmapper)

if("${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION}" LESS 3.1)
	# Use C++11 : cmake <=3.1
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
else()
	# Use C++11 : cmake >= 3.1
	set(CMAKE_CXX_STANDARD 11)
	set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

include_directories(include)

##############
# External Library
##############

# enable_testing()

# TODO(gocarlos) add here the gtest 
# add_subdirectory(lib/googletest-1.8.0)
# include_directories(${gtest_SOURCE_DIR}/include ${gtest_SOURCE_DIR})

# TODO(gocarlos) add here the SLAM engine
# add_subdirectory(thirdparty/engine/ORB_SLAM2)
# include_directories(${ORB_SLAM2_SOURCE_DIR}/include ${ORB_SLAM2_SOURCE_DIR})

##############
# Library
##############

add_library(wrapper_lib src/Wrapper.cpp)

##############
# Executables
##############

add_executable (main src/main.cpp)
target_link_libraries(main wrapper_lib)

##############
# Unit Tests
##############

# add_executable(test_pose gtest/test_pose.cpp)
# target_link_libraries(test_pose gtest gtest_main)