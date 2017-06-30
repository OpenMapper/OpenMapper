CMAKE_MINIMUM_REQUIRED (VERSION 3.1)

################################
# OpenMapper
################################

PROJECT(OpenMapperDesktopApp)
SET(OPENMAPPER_APP "OpenMapperDesktopApp")

################################
# Define Sources and Headers
################################

FILE(GLOB_RECURSE OM_APP_RESOURCES "${CMAKE_CURRENT_LIST_DIR}/*.cc"
                                   "${CMAKE_CURRENT_LIST_DIR}/*.h")

INCLUDE_DIRECTORIES(PUBLIC "${CMAKE_CURRENT_LIST_DIR}")

################################
# Load Dependencies
################################

# Include OpenMapper Library
INCLUDE("${CMAKE_CURRENT_LIST_DIR}/../../CMakeLists.txt")

################################
# Define and Link Executable
################################

ADD_EXECUTABLE(${OPENMAPPER_APP} ${OM_APP_RESOURCES})

TARGET_LINK_LIBRARIES(${OPENMAPPER_APP} debug ${OPENMAPPER_LIB} optimized ${OPENMAPPER_LIB})
