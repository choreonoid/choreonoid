if(NOT BUILD_OPENRTM_PLUGIN)
  return()
endif()

message(STATUS "src/OpenRTM Build" )

if(UNIX)
  if(NOT OPENRTM_DIR)
    pkg_check_modules(OPENRTM REQUIRED openrtm-aist)
    execute_process(
      COMMAND pkg-config --variable=prefix openrtm-aist
      OUTPUT_VARIABLE OPENRTM_DIR
      RESULT_VARIABLE RESULT
      OUTPUT_STRIP_TRAILING_WHITESPACE)
  endif()
endif()

if(OPENRTM_DIR)
  set(OPENRTM_INCLUDE_DIRS
    ${OPENRTM_DIR}/include/coil-${OPENRTM_VERSION_MAJOR}.${OPENRTM_VERSION_MINOR}
    ${OPENRTM_DIR}/include/openrtm-${OPENRTM_VERSION_MAJOR}.${OPENRTM_VERSION_MINOR}
    ${OPENRTM_DIR}/include/openrtm-${OPENRTM_VERSION_MAJOR}.${OPENRTM_VERSION_MINOR}/rtm/idl
    ${OPENRTM_DIR}/rtm/idl
    ${OPENRTM_DIR}
    )
endif()

# Make the following variables accessible from other directories
set(OPENRTM_DEFINITIONS ${OPENRTM_DEFINITIONS} PARENT_SCOPE)
add_definitions(${OPENRTM_DEFINITIONS})
include_directories(${OPENRTM_INCLUDE_DIRS})
link_directories(${OPENRTM_LIBRARY_DIRS})


# Port data type stubs
set(IDL_INCLUDE_DIRS 
  ${OPENRTM_DIR}/include/openrtm-${OPENRTM_VERSION_MAJOR}.${OPENRTM_VERSION_MINOR}/rtm/idl
  ${OPENRTM_DIR}/rtm/idl # for the Windows installer version
  )

# BodyIoRTC
set(target2 CnoidBodyIoRTC)
set(sources BodyIoRTC.cpp)
set(headers BodyIoRTC.h exportdecl.h)

if(MSVC)
  add_definitions(-DRTC_CORBA_CXXMAPPING11)
endif()

add_cnoid_library(${target2} SHARED ${sources} ${headers})
target_link_libraries(${target2} CnoidBody CnoidCorba ${OPENRTM_LIBRARIES})
apply_common_setting_for_library(${target2} "${headers}")


function(add_cnoid_body_io_rtc)

  set(target ${ARGV0})
  list(REMOVE_AT ARGV 0)

  add_library(${target} SHARED ${ARGV})
  target_link_libraries(${target} CnoidBodyIoRTC)
  
  if(MSVC)
    add_definitions(-DRTC_CORBA_CXXMAPPING11)
  endif()

  set_target_properties(${target} PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/${CNOID_PLUGIN_SUBDIR}/rtc
    PREFIX "")
  if(ENABLE_INSTALL_RPATH)
    set_target_properties(${target} PROPERTIES INSTALL_RPATH "$ORIGIN/../..")
  endif()

  install(TARGETS ${target}
    RUNTIME DESTINATION ${CNOID_PLUGIN_SUBDIR}/rtc CONFIGURATIONS Release Debug
    LIBRARY DESTINATION ${CNOID_PLUGIN_SUBDIR}/rtc CONFIGURATIONS Release Debug)

endfunction()
