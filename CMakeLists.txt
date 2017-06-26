cmake_minimum_required (VERSION 3.1)

################################
# OpenMapper`
################################

PROJECT(OpenMapper)
SET(OM_WRAPPER_LIB "OpenMapper")
SET(OM_WRAPPER_APP "OpenMapperApp")

SET(CMAKE_CXX_STANDARD 11)
SET(CMAKE_CXX_STANDARD_REQUIRED ON)

SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -w")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -w")

SET(BUILD_TESTS TRUE CACHE BOOL "Build OpenMapper tests")
SET(BUILD_APP FALSE CACHE BOOL "Build OpenMapper App")
SET(BUILD_IOS_FW FALSE CACHE BOOL "Build OpenMapper Lib as iOS Framework")

################################
# Define Sources and Headers
################################

FILE(GLOB_RECURSE OM_LIB_HEADERS "${CMAKE_CURRENT_LIST_DIR}/include/wrapper.h")
FILE(GLOB_RECURSE OM_LIB_SOURCES "${CMAKE_CURRENT_LIST_DIR}/src/wrapper.cc")

IF(BUILD_IOS_FW)
  SET(OM_LIB_RESOURCES ${OM_LIB_SOURCES} "${CMAKE_CURRENT_LIST_DIR}/wrapper/ios/open_mapper.mm"
                                         "${CMAKE_CURRENT_LIST_DIR}/wrapper/ios/open_mapper.h")
ENDIF(BUILD_IOS_FW)
SET(OM_APP_RESOURCES "${CMAKE_CURRENT_LIST_DIR}/src/app/main.cc")

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

INCLUDE_DIRECTORIES(PUBLIC "${CMAKE_CURRENT_LIST_DIR}/include")


################################
# Define Dependencies
################################

FIND_PACKAGE(OpenCV 3.0.0 REQUIRED)
FIND_PACKAGE(Eigen3 3.1.0 REQUIRED)
FIND_PACKAGE(OpenGL REQUIRED)
# FIND_LIBRARY(OpenGL_LIBRARIES OpenGL)
# SET(OpenCV_DEBUG_LIBRARIES ${OpenCV_LIBRARIES})
INCLUDE_DIRECTORIES("/usr/local/include")

INCLUDE("${CMAKE_CURRENT_LIST_DIR}/thirdparty/slam_engine/ORB_SLAM2/CMakeLists.txt")

INCLUDE_DIRECTORIES(
  ${CMAKE_CURRENT_LIST_DIR}/thirdparty/slam_engine/ORB_SLAM2/include
  ${CMAKE_CURRENT_LIST_DIR}/thirdparty/slam_engine/ORB_SLAM2/
  ${CMAKE_CURRENT_LIST_DIR}/thirdparty/slam_engine/ORB_SLAM2/Thirdparty/DBoW2/DBoW2
  ${CMAKE_CURRENT_LIST_DIR}/thirdparty/slam_engine/ORB_SLAM2/Thirdparty/g2o/g2o/core
  ${CMAKE_CURRENT_LIST_DIR}/thirdparty/slam_engine/ORB_SLAM2/Thirdparty/g2o/g2o/solvers
  ${CMAKE_CURRENT_LIST_DIR}/thirdparty/slam_engine/ORB_SLAM2/Thirdparty/g2o/g2o/stuff
  ${CMAKE_CURRENT_LIST_DIR}/thirdparty/slam_engine/ORB_SLAM2/Thirdparty/g2o/g2o/types
  ${EIGEN3_INCLUDE_DIR}
)


################################
# Define OpenMapper Library
################################
IF(BUILD_IOS_FW)
  INCLUDE(cmake_modules/ios_10_3_config.cmake)
  INCLUDE(cmake_modules/xcode_framework_config.cmake)

  ADD_LIBRARY(${OM_WRAPPER_LIB} SHARED ${OM_LIB_RESOURCES})
  # Configure build targets for iOS
  CONFIGURE_FOR_IOS_10_3(${OM_WRAPPER_LIB})
  CONFIGURE_FOR_IOS_10_3(${ORB_SLAM2_LIB})
  CONFIGURE_FOR_IOS_10_3(${DBOW2_LIB})
  CONFIGURE_FOR_IOS_10_3(${G2O_LIB})

  CONFIGURE_FOR_XCODE_FW(${OM_WRAPPER_LIB} "${CMAKE_CURRENT_LIST_DIR}/wrapper/ios/open_mapper.h")

  TARGET_LINK_LIBRARIES(${OM_WRAPPER_LIB}
    debug ${ORB_SLAM2_LIB} optimized ${ORB_SLAM2_LIB}
    debug ${DBOW2_LIB} optimized ${DBOW2_LIB}
    debug ${G2O_LIB} debug ${G2O_LIB}
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
ELSE(BUILD_IOS_FW)
  ADD_LIBRARY(${OM_WRAPPER_LIB} STATIC ${OM_LIB_RESOURCES})

  TARGET_LINK_LIBRARIES(${OM_WRAPPER_LIB}
  # debug ${OpenGL_LIBRARIES} optimized ${OpenGL_LIBRARIES}
    debug ${OpenCV_LIBRARIES} optimized ${OpenCV_LIBRARIES}
    debug ${ORB_SLAM2_LIB} optimized ${ORB_SLAM2_LIB}
    debug ${DBOW2_LIB} optimized ${DBOW2_LIB}
    debug ${G2O_LIB} debug ${G2O_LIB}
  )
ENDIF(BUILD_IOS_FW)
SET_TARGET_PROPERTIES(${OM_WRAPPER_LIB} PROPERTIES
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
# Define OpenMapper App
################################

IF(BUILD_APP)
  ADD_EXECUTABLE(${OM_WRAPPER_APP} ${OM_APP_RESOURCES})

  SET_TARGET_PROPERTIES(${OM_WRAPPER_APP} PROPERTIES
      DEBUG_POSTFIX "_d"
      RUNTIME_OUTPUT_DIRECTORY ../bin/
      RUNTIME_OUTPUT_DIRECTORY_DEBUG ../bin/
      RUNTIME_OUTPUT_DIRECTORY_RELEASE ../bin/
      RUNTIME_OUTPUT_DIRECTORY_RELWITHDEBINFO ../bin/)

  TARGET_LINK_LIBRARIES(${OM_WRAPPER_APP}
    # debug ${OpenGL_LIBRARIES} optimized ${OpenGL_LIBRARIES}
    debug ${OpenCV_LIBRARIES} optimized ${OpenCV_LIBRARIES}
    debug ${OM_WRAPPER_LIB} optimized ${OM_WRAPPER_LIB}
    debug ${ORB_SLAM2_LIB} optimized ${ORB_SLAM2_LIB}
    debug ${DBOW2_LIB} optimized ${DBOW2_LIB}
    debug ${G2O_LIB} optimized ${G2O_LIB}
  )
ENDIF(BUILD_APP)

################################
# OpenMapper Unit Tests
################################

IF(BUILD_TESTS)
  INCLUDE("${CMAKE_CURRENT_LIST_DIR}/test/CMakeLists.txt")
ENDIF(BUILD_TESTS)

# # Uncomment to display all CMAKE variables:
# GET_CMAKE_PROPERTY(VARIABLE_NAMES VARIABLES)
# FOREACH (VARIABLE_NAME ${VARIABLE_NAMES})
#    MESSAGE(STATUS "${VARIABLE_NAME}=${${VARIABLE_NAME}}")
# ENDFOREACH()
