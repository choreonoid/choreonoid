if(NOT ENABLE_GUI)
  return()
endif()

option(BUILD_MULTICOPTER_SAMPLE "Building Multicopter Sample" OFF)
if(NOT BUILD_MULTICOPTER_SAMPLE)
  return()
endif()

if(NOT BUILD_OPENRTM_PLUGIN)
  message(FATAL_ERROR "MulticopterSample need to build OpenRTMPlugin.")
endif()

if(NOT BUILD_MULTICOPTER_PLUGIN)
  message(FATAL_ERROR "MulticopterRTM need to build Multicopterlugin.")
endif()

if(NOT BUILD_OPENRTM_SAMPLES)
  message(FATAL_ERROR "MulticopterRTM need to build OpenRTM Samples")
endif()

set(CMAKE_BUILD_TYPE Debug)

include_directories(${OPENRTM_INCLUDE_DIRS})
link_directories(${OPENRTM_LIBRARY_DIRS})
if(WIN32)
  add_definitions(-D__WIN32__ -D__x86__ -D__NT__ -D__OSVERSION__=4 -D_CRT_SECURE_NO_DEPRECATE -D_WIN32_WINNT=0x0500 -DRTC_CORBA_CXXMAPPING11)
endif()

# Multicopter Controller
add_cnoid_simple_controller(MulticopterController MulticopterController.cpp)
configure_file(MulticopterJoystick.cnoid ${CNOID_SOURCE_SHARE_DIR}/project COPYONLY)

# Multicopter Controller
set(target1 MulticopterControllerRTC)
add_library(${target1} SHARED MulticopterControllerRTC.cpp)
target_link_libraries(${target1} CnoidBodyIoRTC)
set_target_properties(${target1} PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/${CNOID_PLUGIN_SUBDIR}/rtc PREFIX "")
install(TARGETS ${target1}
  RUNTIME DESTINATION ${CNOID_PLUGIN_SUBDIR}/rtc CONFIGURATIONS Release Debug
  LIBRARY DESTINATION ${CNOID_PLUGIN_SUBDIR}/rtc CONFIGURATIONS Release Debug)

# Multicopter Io
set(target2 MulticopterIoRTC)
add_cnoid_body_io_rtc(${target2} MulticopterIoRTC.cpp)

configure_file(OpenRTM-MulticopterJoystick.cnoid ${CNOID_SOURCE_SHARE_DIR}/project COPYONLY)
