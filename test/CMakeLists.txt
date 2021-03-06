################################
# Google Test Setup
################################

# When building OpenMapper with ROS, gtest_main & gtest are already targets which are introduced automatically by catkin.
# Here gtest is only built if there is not target called gtest_main, which means catkin is not providing already this target.
# Similiar problem: https://stackoverflow.com/questions/42550873/compile-gtest-from-source-with-catkin
# See documentation for policy CMP0002.
IF(NOT TARGET ${gtest_main})
  IF(NOT BUILD_ROS_BINDING)
    ADD_SUBDIRECTORY(${CMAKE_CURRENT_LIST_DIR}/../thirdparty/googletest/googletest gtest_build)
    ENABLE_TESTING()
    INCLUDE_DIRECTORIES(${gtest_SOURCE_DIR}/include ${gtest_SOURCE_DIR})
  ENDIF(NOT BUILD_ROS_BINDING)
ENDIF(NOT TARGET ${gtest_main})

################################
# OpenMapper Tests
################################

SET(OM_WRAPPER_TESTS "OpenMapperTests")

FILE(GLOB_RECURSE OM_WRAPPER_TESTS_HEADERS "${CMAKE_CURRENT_LIST_DIR}/*.h")
FILE(GLOB_RECURSE OM_WRAPPER_TESTS_SOURCES "${CMAKE_CURRENT_LIST_DIR}/*.cc")

# Add test src files
ADD_EXECUTABLE(${OM_WRAPPER_TESTS}
  ${OM_WRAPPER_TESTS_HEADERS}
  ${OM_WRAPPER_TESTS_SOURCES}
)

# Link test executable against gtest & gtest_main
TARGET_LINK_LIBRARIES(${OM_WRAPPER_TESTS}
  gtest gtest_main
  debug ${OPENMAPPER_LIB} optimized ${OPENMAPPER_LIB}
)

ADD_TEST(${OM_WRAPPER_TESTS} ${OM_WRAPPER_TESTS})

TARGET_INCLUDE_DIRECTORIES(${OM_WRAPPER_TESTS} PUBLIC include)

SET_TARGET_PROPERTIES(${OM_WRAPPER_TESTS} PROPERTIES
  DEBUG_POSTFIX "_d"
  RUNTIME_OUTPUT_DIRECTORY ../bin/
  RUNTIME_OUTPUT_DIRECTORY_DEBUG ../bin/
  RUNTIME_OUTPUT_DIRECTORY_RELEASE ../bin/
  RUNTIME_OUTPUT_DIRECTORY_RELWITHDEBINFO ../bin/
)
