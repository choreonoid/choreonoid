
#set(CMAKE_BUILD_TYPE Debug)

if(NOT ENABLE_GUI)
  return()
endif()

option(BUILD_VISION_SENSOR_RTM_SAMPLE "Building a sample for visualizating vision sensor data using OpenRTM" OFF)

if(NOT BUILD_VISION_SENSOR_RTM_SAMPLE)
  return()
else()
  if(NOT BUILD_OPENRTM_PLUGIN)
    message(FATAL_ERROR "BUILD_VISION_SENSOR_RTM_SAMPLE requires BUILD_OPENRTM_PLUGIN")
  endif()
  if(NOT BUILD_OPENRTM_SAMPLES)
    message(FATAL_ERROR "BUILD_VISION_SENSOR_RTM_SAMPLE requires BUILD_OPENRTM_SAMPLES")
  endif()
endif()

set(sources VisionSensorRTMSamplePlugin.cpp)

set(target CnoidVisionSensorRTMSamplePlugin)
include_directories(${OPENRTM_INCLUDE_DIRS})
link_directories(${OPENRTM_LIBRARY_DIRS})

add_cnoid_plugin(${target} SHARED ${sources})

if(MSVC)
add_definitions(-D__WIN32__ -D__x86__ -D__NT__ -D__OSVERSION__=4 -D_CRT_SECURE_NO_DEPRECATE -D_WIN32_WINNT=0x0500 -DRTC_CORBA_CXXMAPPING11)
endif()

if(MSVC)
  target_link_libraries(${target} CnoidBodyPlugin CnoidCorbaPlugin ${OPENRTM_LIBRARIES} optimized CnoidOpenRTMPlugin debug  CnoidOpenRTMd)
else()
  target_link_libraries(${target} CnoidBodyPlugin CnoidCorbaPlugin ${OPENRTM_LIBRARIES} CnoidOpenRTMPlugin)
endif()
apply_common_setting_for_plugin(${target})

configure_file(OpenRTM-TankJoystickVisionSensors.cnoid ${CNOID_SOURCE_SHARE_DIR}/project COPYONLY)
configure_file(TankVisionSensors.conf
  ${PROJECT_BINARY_DIR}/${CNOID_PLUGIN_SUBDIR}/rtc/TankVisionSensors.conf COPYONLY)
install(FILES TankVisionSensors.conf DESTINATION ${CNOID_PLUGIN_SUBDIR}/rtc)
configure_file(OpenRTM-TankJoystickVisionSensors-RTSystem.cnoid ${CNOID_SOURCE_SHARE_DIR}/project COPYONLY)
configure_file(TankVisionSensors_Noconnection.conf
  ${PROJECT_BINARY_DIR}/${CNOID_PLUGIN_SUBDIR}/rtc/TankVisionSensors_Noconnection.conf COPYONLY)
install(FILES TankVisionSensors_Noconnection.conf DESTINATION ${CNOID_PLUGIN_SUBDIR}/rtc)
configure_file(TankJoystickPointCloudIO.cnoid ${CNOID_SOURCE_SHARE_DIR}/project COPYONLY)
