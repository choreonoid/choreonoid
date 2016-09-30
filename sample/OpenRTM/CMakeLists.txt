
# @author Shizuko Hattori

if(NOT ENABLE_GUI)
  return()
endif()

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

  target_link_libraries(${target} CnoidCorba CnoidBody ${OPENRTM_LIBRARIES})

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

# SR1 walk
set(target SR1WalkControllerRTC)
add_cnoid_sample_rtc(${target} SR1WalkControllerRTC.cpp)
configure_file(OpenRTM-SR1Walk.cnoid ${CNOID_SOURCE_SHARE_DIR}/project COPYONLY)
  
# SR1 lifting up
set(target SR1LiftupControllerRTC)
add_cnoid_sample_rtc(${target} SR1LiftupControllerRTC.cpp)
configure_file(OpenRTM-SR1Liftup.cnoid ${CNOID_SOURCE_SHARE_DIR}/project COPYONLY)

# SR1 lifting up AgX high-gain
if(BUILD_AGX_PLUGIN)
  set(target SR1LiftupHGControllerRTC)
  add_cnoid_sample_rtc(${target} SR1LiftupHGControllerRTC.cpp)
  configure_file(OpenRTM-SR1LiftupHG.cnoid ${CNOID_SOURCE_SHARE_DIR}/project COPYONLY)
  configure_file(SR1LiftupHG.conf ${PROJECT_BINARY_DIR}/${CNOID_PLUGIN_SUBDIR}/rtc/SR1LiftupHG.conf COPYONLY)
  install(FILES SR1LiftupHG.conf DESTINATION ${CNOID_PLUGIN_SUBDIR}/rtc)
endif()

# PA10 Pickup
set(target PA10PickupControllerRTC)
add_cnoid_sample_rtc(${target} PA10PickupControllerRTC.cpp)
configure_file(OpenRTM-PA10Pickup.cnoid ${CNOID_SOURCE_SHARE_DIR}/project COPYONLY)
configure_file(PA10Pickup.conf ${PROJECT_BINARY_DIR}/${CNOID_PLUGIN_SUBDIR}/rtc/PA10Pickup.conf COPYONLY)
install(FILES PA10Pickup.conf DESTINATION ${CNOID_PLUGIN_SUBDIR}/rtc)

# Joystick
add_cnoid_sample_rtc(JoystickRTC JoystickRTC.cpp)
target_link_libraries(JoystickRTC CnoidUtil)

# Sample Crawler Joystick Controller
set(target SampleCrawlerJoystickControllerRTC)
add_cnoid_sample_rtc(${target} SampleCrawlerJoystickControllerRTC.cpp)
configure_file(OpenRTM-SampleCrawlerJoystick.cnoid ${CNOID_SOURCE_SHARE_DIR}/project COPYONLY)
configure_file(SampleCrawlerJoystick.conf
  ${PROJECT_BINARY_DIR}/${CNOID_PLUGIN_SUBDIR}/rtc/SampleCrawlerJoystick.conf COPYONLY)
install(FILES SampleCrawlerJoystick.conf DESTINATION ${CNOID_PLUGIN_SUBDIR}/rtc)

# Tank Joystick Controller
set(target TankJoystickControllerRTC)
add_cnoid_sample_rtc(${target} TankJoystickControllerRTC.cpp)
configure_file(OpenRTM-TankJoystick.cnoid ${CNOID_SOURCE_SHARE_DIR}/project COPYONLY)
configure_file(OpenRTM-TankJoystick-RTSystem.cnoid ${CNOID_SOURCE_SHARE_DIR}/project COPYONLY)
configure_file(TankJoystick.conf
  ${PROJECT_BINARY_DIR}/${CNOID_PLUGIN_SUBDIR}/rtc/TankJoystick.conf COPYONLY)
configure_file(TankJoystick_Noconnection.conf
  ${PROJECT_BINARY_DIR}/${CNOID_PLUGIN_SUBDIR}/rtc/TankJoystick_Noconnection.conf COPYONLY)
install(FILES TankJoystick.conf DESTINATION ${CNOID_PLUGIN_SUBDIR}/rtc)
install(FILES TankJoystick_Noconnection.conf DESTINATION ${CNOID_PLUGIN_SUBDIR}/rtc)

# Tank Kinematics Controller
set(target TankKinematicsControllerRTC)
add_cnoid_sample_rtc(${target} TankKinematicsControllerRTC.cpp)
configure_file(OpenRTM-TankKinematics.cnoid ${CNOID_SOURCE_SHARE_DIR}/project COPYONLY)
configure_file(TankKinematics.conf ${PROJECT_BINARY_DIR}/${CNOID_PLUGIN_SUBDIR}/rtc/TankKinematics.conf COPYONLY)
install(FILES TankKinematics.conf DESTINATION ${CNOID_PLUGIN_SUBDIR}/rtc)

# constraintLog
add_cnoid_sample_rtc(ConstraintLogRTC ConstraintLogRTC.cpp)
configure_file(OpenRTM-SR1LiftupConstLog.cnoid ${CNOID_SOURCE_SHARE_DIR}/project COPYONLY)
configure_file(SR1LiftupConstLog.conf ${PROJECT_BINARY_DIR}/${CNOID_PLUGIN_SUBDIR}/rtc/SR1LiftupConstLog.conf COPYONLY)
install(FILES SR1LiftupConstLog.conf DESTINATION ${CNOID_PLUGIN_SUBDIR}/rtc)
configure_file(SR1LiftupConstLog.py ${CNOID_SOURCE_SHARE_DIR}/script COPYONLY)

