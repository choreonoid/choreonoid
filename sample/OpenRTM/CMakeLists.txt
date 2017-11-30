option(BUILD_OPENRTM_SAMPLES "Building OpenRTM samples" OFF)
if(NOT BUILD_OPENRTM_SAMPLES)
  return()
elseif(NOT BUILD_OPENRTM_PLUGIN)
  message(FATAL_ERROR "OpenRTM samples need to build OpenRTMPlugin.")
endif()

include_directories(${OPENRTM_INCLUDE_DIRS})
link_directories(${OPENRTM_LIBRARY_DIRS})
if(WIN32)
  add_definitions(-D__WIN32__ -D__x86__ -D__NT__ -D__OSVERSION__=4 -D_CRT_SECURE_NO_DEPRECATE -D_WIN32_WINNT=0x0500 -DRTC_CORBA_CXXMAPPING11)
endif()

function(add_cnoid_sample_rtc)

  set(target ${ARGV0})
  list(REMOVE_AT ARGV 0)

  add_library(${target} SHARED ${ARGV})
  target_link_libraries(${target} CnoidBodyIoRTC)

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

# Robot IO
add_cnoid_body_io_rtc(RobotIoRTC RobotIoRTC.cpp)

# SR1 walk
add_cnoid_sample_rtc(SR1WalkControllerRTC SR1WalkControllerRTC.cpp)
configure_file(OpenRTM-SR1Walk.cnoid ${CNOID_SOURCE_SHARE_DIR}/project COPYONLY)
  
# SR1 lifting up
add_cnoid_sample_rtc(SR1LiftupControllerRTC SR1LiftupControllerRTC.cpp)
configure_file(OpenRTM-SR1Liftup.cnoid ${CNOID_SOURCE_SHARE_DIR}/project COPYONLY)

# PA10 Pickup
add_cnoid_sample_rtc(PA10PickupControllerRTC PA10PickupControllerRTC.cpp)
configure_file(OpenRTM-PA10Pickup.cnoid ${CNOID_SOURCE_SHARE_DIR}/project COPYONLY)

# Tank IO
add_cnoid_body_io_rtc(TankIoRTC TankIoRTC.cpp)
add_cnoid_body_io_rtc(TankVisionSensorIoRTC TankVisionSensorIoRTC.cpp)

# Joystick
add_cnoid_sample_rtc(JoystickRTC JoystickRTC.cpp)
target_link_libraries(JoystickRTC CnoidUtil)

# Tank Joystick Controller
set(target TankJoystickControllerRTC)
add_cnoid_sample_rtc(${target} TankJoystickControllerRTC.cpp)
configure_file(OpenRTM-TankJoystick.cnoid ${CNOID_SOURCE_SHARE_DIR}/project COPYONLY)
configure_file(OpenRTM-TankVisionSensors.cnoid ${CNOID_SOURCE_SHARE_DIR}/project COPYONLY)
