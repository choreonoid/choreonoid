option(BUILD_OPENRTM_SAMPLES "Building OpenRTM samples" OFF)
if(NOT BUILD_OPENRTM_SAMPLES)
  return()
elseif(NOT BUILD_OPENRTM_PLUGIN)
  message(FATAL_ERROR "OpenRTM samples need to build OpenRTMPlugin.")
endif()

add_definitions(${OPENRTM_DEFINITIONS})
include_directories(${OPENRTM_INCLUDE_DIRS})
link_directories(${OPENRTM_LIBRARY_DIRS})

if(MSVC)
  add_compile_options(${OPENRTM_CFLAGS})
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

set(project_files "")

# General BodyIoRTCs
add_cnoid_body_io_rtc(RobotIoRTC RobotIoRTC.cpp)

# SR1 walk
add_cnoid_sample_rtc(SR1WalkControllerRTC SR1WalkControllerRTC.cpp)
list(APPEND project_files OpenRTM-SR1Walk.cnoid)
  
# SR1 lifting up
add_cnoid_sample_rtc(SR1LiftupControllerRTC SR1LiftupControllerRTC.cpp)
list(APPEND project_files OpenRTM-SR1Liftup.cnoid)

# PA10 Pickup
add_cnoid_sample_rtc(PA10PickupControllerRTC PA10PickupControllerRTC.cpp)
list(APPEND project_files OpenRTM-PA10Pickup.cnoid)

# Tank IO
add_cnoid_body_io_rtc(TankIoRTC TankIoRTC.cpp)

# Joystick
add_cnoid_sample_rtc(JoystickRTC JoystickRTC.cpp)
target_link_libraries(JoystickRTC CnoidUtil)
add_cnoid_sample_rtc(RemoteJoystickRTC RemoteJoystickRTC.cpp)
target_link_libraries(RemoteJoystickRTC CnoidUtil)

add_cnoid_simple_controller(RemoteJoystickInputController RemoteJoystickInputController.cpp)
target_link_libraries(RemoteJoystickInputController CnoidBody CnoidOpenRTMPlugin ${OPENRTM_LIBRARIES})

# Tank Joystick Controller
set(target TankJoystickControllerRTC)
add_cnoid_sample_rtc(${target} TankJoystickControllerRTC.cpp)
list(APPEND project_files OpenRTM-Tank.cnoid)

if(NOT USE_BUILTIN_CAMERA_IMAGE_IDL)
  add_cnoid_body_io_rtc(VisionSensorIoRTC VisionSensorIoRTC.cpp)
  target_link_libraries(VisionSensorIoRTC ${OPENRTM_CAMERA_LIBRARY})
  list(APPEND project_files
    OpenRTM-TankVisionSensors.cnoid
    TankVisionSystem.xml
    OpenRTM-TankVisionSensors-NewIDL.cnoid
    TankVisionSystem-NewIDL.xml
    )
endif()

# Teleoperation samples
list(APPEND project_files
  OpenRTM-AizuSpiderSS-Remote.cnoid
  OpenRTM-AizuSpider-Terminal.cnoid
  OpenRTM-DoubleArmV7-Terminal.cnoid)

install(FILES ${project_files} DESTINATION ${CNOID_SHARE_SUBDIR}/project)
