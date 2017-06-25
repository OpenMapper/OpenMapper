FUNCTION(CONFIGURE_FOR_XCODE_FW BUILD_TARGET PUBLIC_HEADERS)
  SET_TARGET_PROPERTIES(${BUILD_TARGET} PROPERTIES
    FRAMEWORK TRUE
    FRAMEWORK_VERSION C
    MACOSX_FRAMEWORK_IDENTIFIER com.kapanu.${BUILD_TARGET}
    # MACOSX_FRAMEWORK_INFO_PLIST Info.plist
    PUBLIC_HEADER "${PUBLIC_HEADERS}"
    # XCODE_ATTRIBUTE_GCC_PREFIX_HEADER "${CMAKE_CURRENT_SOURCE_DIR}/Prefix.pch"
    XCODE_ATTRIBUTE_FRAMEWORK_SEARCH_PATHS "${CMAKE_CURRENT_LIST_DIR}/thirdparty/bin"
    XCODE_ATTRIBUTE_CODE_SIGNING_ALLOWED "NO"
    XCODE_ATTRIBUTE_GCC_SYMBOLS_PRIVATE_EXTERN NO
    # MACOSX_BUNDLE_SHORT_VERSION_STRING ${OPENMAPPER_VERSION}
    # MACOSX_BUNDLE_BUNDLE_VERSION ${OPENMAPPER_VERSION}
    MACOSX_BUNDLE_INFO_STRING "OpenMapper dyn lib bundled as Framework for iOS"
    MACOSX_BUNDLE_BUNDLE_NAME ${BUILD_TARGET}
    # MACOSX_BUNDLE_COPYRIGHT ${OPENMAPPER_COPYRIGHT_NOTE}
    # MACOSX_BUNDLE_INFO_PLIST ${CMAKE_CURRENT_SOURCE_DIR}/Info.plist.in
  )
ENDFUNCTION(CONFIGURE_FOR_XCODE_FW)