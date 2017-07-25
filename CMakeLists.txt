CMAKE_MINIMUM_REQUIRED(VERSION 3.1)

################################
# OpenMapper
################################

SET(OPENMAPPER_LIB "OpenMapper")

OPTION(DO_COVERAGE_TEST "Test the coverage of the unit tests using CodeCov" OFF)

IF(NOT PROJECT_NAME)
  PROJECT(${OPENMAPPER_LIB})
ENDIF(NOT PROJECT_NAME)

# Request C++11 standard from compiler.
SET(CMAKE_CXX_STANDARD 11)
SET(CMAKE_CXX_STANDARD_REQUIRED ON)

SET(CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/cmake_modules)
INCLUDE(FindEigen3)
INCLUDE(ios_10_3_config)
INCLUDE(xcode_framework_config)

# Suppress warnings from the compiler.
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -w")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -w")

IF(DO_COVERAGE_TEST)
  MESSAGE("\nGoing to build the coverage tests.\n\n\n")
  INCLUDE(CodeCoverage)
  SETUP_TARGET_FOR_COVERAGE(${PROJECT_NAME}_coverage tests coverage)
  SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g -O0 -fprofile-arcs -ftest-coverage")
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0 -fprofile-arcs -ftest-coverage")
ELSE()
  MESSAGE("\nNot building the coverage tests. \n\n\n")

  # Optimize the code with the compiler if the target device has a x86 processor and not doing code coverage tests.
  IF(CMAKE_SYSTEM_PROCESSOR MATCHES "amd64.*|x86_64.*|AMD64.*|i686.*|i386.*|x86.*")
    MESSAGE("Current System is x86.")
    SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -march=native")
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native")
  ENDIF()

  IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE Release)
  ENDIF()

ENDIF()

SET(BUILD_TESTS FALSE CACHE BOOL "Build OpenMapper tests.")


################################
# Define Sources and Headers
################################

FILE(GLOB_RECURSE OM_LIB_HEADERS "${CMAKE_CURRENT_LIST_DIR}/include/*.h")
FILE(GLOB_RECURSE OM_LIB_SOURCES "${CMAKE_CURRENT_LIST_DIR}/src/*.cc")

INCLUDE_DIRECTORIES(PUBLIC "${CMAKE_CURRENT_LIST_DIR}/include")

################################
# Define Dependencies
################################

FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(OpenGL REQUIRED)
FIND_PACKAGE(Threads REQUIRED)

INCLUDE_DIRECTORIES(
  ${CMAKE_CURRENT_LIST_DIR}/thirdparty/eigen

  ${OPENGL_INCLUDE_DIR}

  ${CMAKE_CURRENT_LIST_DIR}/thirdparty/miniglog

  ${CMAKE_CURRENT_LIST_DIR}/thirdparty/slam_engine/ORB_SLAM2/include
  ${CMAKE_CURRENT_LIST_DIR}/thirdparty/slam_engine/ORB_SLAM2/
  ${CMAKE_CURRENT_LIST_DIR}/thirdparty/slam_engine/ORB_SLAM2/Thirdparty/DBoW2/DBoW2
  ${CMAKE_CURRENT_LIST_DIR}/thirdparty/slam_engine/ORB_SLAM2/Thirdparty/g2o/g2o/core
  ${CMAKE_CURRENT_LIST_DIR}/thirdparty/slam_engine/ORB_SLAM2/Thirdparty/g2o/g2o/solvers
  ${CMAKE_CURRENT_LIST_DIR}/thirdparty/slam_engine/ORB_SLAM2/Thirdparty/g2o/g2o/stuff
  ${CMAKE_CURRENT_LIST_DIR}/thirdparty/slam_engine/ORB_SLAM2/Thirdparty/g2o/g2o/types
)

INCLUDE("${CMAKE_CURRENT_LIST_DIR}/thirdparty/miniglog/CMakeLists.txt")
INCLUDE("${CMAKE_CURRENT_LIST_DIR}/thirdparty/slam_engine/ORB_SLAM2/CMakeLists.txt")


################################
# Define OpenMapper Library
################################

ADD_LIBRARY(${OPENMAPPER_LIB} STATIC ${OM_LIB_SOURCES})

TARGET_LINK_LIBRARIES(${OPENMAPPER_LIB}
  Threads::Threads #TODO(gocarlos) check if this is needed.

  debug ${MINI_GLOG} optimized ${MINI_GLOG}
  debug ${OpenCV_LIBRARIES} optimized ${OpenCV_LIBRARIES}
  debug ${DBOW2_LIB} optimized ${DBOW2_LIB}
  debug ${G2O_LIB} optimized ${G2O_LIB}
  debug ${ORB_SLAM2_LIB} optimized ${ORB_SLAM2_LIB}
)

SET_TARGET_PROPERTIES(${OPENMAPPER_LIB} PROPERTIES
  DEBUG_POSTFIX "_d"
  LIBRARY_OUTPUT_DIRECTORY ../lib/
  LIBRARY_OUTPUT_DIRECTORY_DEBUG ../lib/
  LIBRARY_OUTPUT_DIRECTORY_RELEASE ../lib/
  LIBRARY_OUTPUT_DIRECTORY_RELWITHDEBINFO ../lib/
  ARCHIVE_OUTPUT_DIRECTORY ../lib/
  ARCHIVE_OUTPUT_DIRECTORY_DEBUG ../lib/
  ARCHIVE_OUTPUT_DIRECTORY_RELWITHDEBINFO ../lib/
  ARCHIVE_OUTPUT_DIRECTORY_RELEASE ../lib/
)

################################
# OpenMapper Unit Tests
################################

IF(BUILD_TESTS)
  INCLUDE("${CMAKE_CURRENT_LIST_DIR}/test/CMakeLists.txt")
ENDIF(BUILD_TESTS)

################################
# OpenMapper Checks
################################

INCLUDE(Settings)

# Uncomment to display all CMAKE variables:
# GET_CMAKE_PROPERTY(VARIABLE_NAMES VARIABLES)
# FOREACH (VARIABLE_NAME ${VARIABLE_NAMES})
#    MESSAGE(STATUS "${VARIABLE_NAME}=${${VARIABLE_NAME}}")
# ENDFOREACH()
