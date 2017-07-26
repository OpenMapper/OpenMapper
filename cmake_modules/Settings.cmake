

IF(CREATE_SNAP_APP)
  IF(EXISTS "/usr/local/openmapper/Vocabulary/iphone.yaml")
    MESSAGE("Vocabulary file exists.")
  ELSE()
    # MESSAGE(FATAL_ERROR "Did not find the vocabulary file")
  ENDIF()
  SET(INSTALL_DIRECTORE_SETTINGS "/snap/openmapper-desktop2/current/openmapper/Vocabulary")
  MESSAGE("Going to move the settings files to ${CMAKE_INSTALL_PREFIX}/openmapper/Vocabulary/")
  FILE(WRITE ${CMAKE_CURRENT_LIST_DIR}/../include/openmapper/settings.h
    "// Auto-generated headers file, do not edit, will be overwritten by cmake.
    // (c) 2017 OpenMapper\n
    #ifndef INCLUDE_OPENMAPPER_CONFIG_H_
    #define INCLUDE_OPENMAPPER_CONFIG_H_

    #include <string>

    const std::string path_to_vocabulary =
    \"${INSTALL_DIRECTORE_SETTINGS}/orbVoc.bin\";
    const std::string path_to_settings =
    \"${INSTALL_DIRECTORE_SETTINGS}/iphone.yaml\";

    const std::string static_video = \"${INSTALL_DIRECTORE_SETTINGS}/test_data/static.mov\";
    const std::string dynamic_video = \"${INSTALL_DIRECTORE_SETTINGS}/test_data/dynamic.mov\";\n

  #endif  // INCLUDE_OPENMAPPER_CONFIG_H_\n
")
ELSE()
  SET(PATH_TO_SETTINGS
  "${CMAKE_CURRENT_LIST_DIR}/../thirdparty/slam_engine/ORB_SLAM2/Vocabulary")
  MESSAGE("Path to settings is: " ${PATH_TO_SETTINGS})

  IF(EXISTS "${PATH_TO_SETTINGS}/iphone.yaml")
    MESSAGE("Vocabulary file exists.")
  ELSE()
    MESSAGE(FATAL_ERROR "Did not find the vocabulary file")
  ENDIF()

  FILE(WRITE ${CMAKE_CURRENT_LIST_DIR}/../include/openmapper/settings.h
    "// Auto-generated headers file, do not edit, will be overwritten by cmake.
    // (c) 2017 OpenMapper\n
    #ifndef INCLUDE_OPENMAPPER_CONFIG_H_
    #define INCLUDE_OPENMAPPER_CONFIG_H_

    #include <string>

    const std::string path_to_vocabulary =
    \"${PATH_TO_SETTINGS}/orbVoc.bin\";
    const std::string path_to_settings =
    \"${PATH_TO_SETTINGS}/iphone.yaml\";

    const std::string static_video = \"${PATH_TO_SETTINGS}/test_data/static.mov\";
    const std::string dynamic_video = \"${PATH_TO_SETTINGS}/test_data/dynamic.mov\";\n

  #endif  // INCLUDE_OPENMAPPER_CONFIG_H_\n
")
ENDIF(CREATE_SNAP_APP)
