CMAKE_MINIMUM_REQUIRED (VERSION 3.1)

################################
# OpenMapper
################################

PROJECT("OMKit")

SET(OPENMAPPER_IOS_FW "OMKit")

################################
# Define Sources and Headers
################################

SET(OM_IOS_WRAPPER_SOURCES "${CMAKE_CURRENT_LIST_DIR}/open_mapper.mm"
                           "${CMAKE_CURRENT_LIST_DIR}/open_mapper.h")

INCLUDE_DIRECTORIES(PUBLIC "${CMAKE_CURRENT_LIST_DIR}")

################################
# Define Dependencies
################################

# Include OpenMapper lib
INCLUDE("${CMAKE_CURRENT_LIST_DIR}/../../CMakeLists.txt")

# override some flags from OpenMapper Lib
SET(CMAKE_C_FLAGS "-w -std=c++11")
SET(CMAKE_CXX_FLAGS "-w -std=c++11")

################################
# Define OpenMapper Library
################################

INCLUDE("${CMAKE_CURRENT_LIST_DIR}/../../cmake_modules/ios_10_3_config.cmake")
INCLUDE("${CMAKE_CURRENT_LIST_DIR}/../../cmake_modules/xcode_framework_config.cmake")

ADD_LIBRARY(${OPENMAPPER_IOS_FW} SHARED ${OM_IOS_WRAPPER_SOURCES})

# Configure build targets for iOS.
CONFIGURE_FOR_IOS_10_3(${OPENMAPPER_IOS_FW})
CONFIGURE_FOR_IOS_10_3(${MINI_GLOG})
CONFIGURE_FOR_IOS_10_3(${OPENMAPPER_LIB})
CONFIGURE_FOR_IOS_10_3(${ORB_SLAM2_LIB})
CONFIGURE_FOR_IOS_10_3(${DBOW2_LIB})
CONFIGURE_FOR_IOS_10_3(${G2O_LIB})

CONFIGURE_FOR_XCODE_FW(${OPENMAPPER_IOS_FW} "${CMAKE_CURRENT_LIST_DIR}/open_mapper.h")

TARGET_LINK_LIBRARIES(${OPENMAPPER_IOS_FW}
  debug ${OPENMAPPER_LIB} optimized ${OPENMAPPER_LIB}
  "-framework Accelerate"
  "-framework AVFoundation"
  "-framework AVKit"
  "-framework CoreGraphics"
  "-framework CoreMedia"
  "-framework CoreVideo"
  "-framework GLKit"
  "-framework opencv2"
  "-framework OpenGLES"
  "-framework UIKit"
)

SET_TARGET_PROPERTIES(${OPENMAPPER_IOS_FW} PROPERTIES
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
