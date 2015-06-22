
#set(CMAKE_BUILD_TYPE Debug)

if(NOT ENABLE_GUI)
  return()
endif()

option(BUILD_VISION_SENSOR_SAMPLE "Building a sample plugin for visualizating vision sensor data" OFF)
if(NOT BUILD_VISION_SENSOR_SAMPLE)
  return()
endif()

set(sources VisionSensorSamplePlugin.cpp)
set(target CnoidVisionSensorSamplePlugin)
add_cnoid_plugin(${target} SHARED ${sources})
target_link_libraries(${target} CnoidBodyPlugin)
apply_common_setting_for_plugin(${target})
configure_file(TankJoystickVisionSensors.cnoid ${CNOID_SOURCE_SHARE_DIR}/project COPYONLY)
