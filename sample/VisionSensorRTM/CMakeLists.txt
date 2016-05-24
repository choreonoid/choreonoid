
#set(CMAKE_BUILD_TYPE Debug)

if(NOT ENABLE_GUI)
  return()
endif()

option(BUILD_RTC_VISION_SENSOR_SAMPLE "Building a rtc sample for visualizating vision sensor data" OFF)

if(NOT BUILD_RTC_VISION_SENSOR_SAMPLE)
  return()
else()
  if(NOT BUILD_OPENRTM_PLUGIN)
    message(FATAL_ERROR "RTC Vision Sensor Sample Plugin requires OpenRTMPlugin.")
  endif()
endif()

set(sources RTCVisionSensorSamplePlugin.cpp)

set(target CnoidRTCVisionSensorSamplePlugin)
include_directories(${OPENRTM_INCLUDE_DIRS})
link_directories(${OPENRTM_LIBRARY_DIRS})

add_cnoid_plugin(${target} SHARED ${sources})

if(MSVC)
add_definitions(-D__WIN32__ -D__x86__ -D__NT__ -D__OSVERSION__=4 -D_CRT_SECURE_NO_DEPRECATE -D_WIN32_WINNT=0x0500 -DRTC_CORBA_CXXMAPPING11)
endif()

if(MSVC)
  target_link_libraries(${target} CnoidBodyPlugin CnoidCorbaPlugin ${OPENRTM_LIBRARIES} optimized CnoidOpenRTM debug  CnoidOpenRTMd)
else()
  target_link_libraries(${target} CnoidBodyPlugin CnoidCorbaPlugin ${OPENRTM_LIBRARIES} CnoidOpenRTM)
endif()
apply_common_setting_for_plugin(${target})

configure_file(OpenRTM-TankJoystickVisionSensor.cnoid ${CNOID_SOURCE_SHARE_DIR}/project COPYONLY)
configure_file(TankVisionSensor.conf
  ${PROJECT_BINARY_DIR}/${CNOID_PLUGIN_SUBDIR}/rtc/TankVisionSensor.conf COPYONLY)
install(FILES TankVisionSensor.conf DESTINATION ${CNOID_PLUGIN_SUBDIR}/rtc)
