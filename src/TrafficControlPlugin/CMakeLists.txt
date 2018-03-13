option(BUILD_TRAFFIC_CONTROL_PLUGIN "Building a Traffic Contorol plugin" OFF)

if(NOT BUILD_TRAFFIC_CONTROL_PLUGIN)
  return()
endif()

set(target CnoidTrafficControlPlugin)

include_directories("${CMAKE_CURRENT_SOURCE_DIR}")
file(GLOB INC
  "${CMAKE_CURRENT_SOURCE_DIR}/*.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/*.hpp")

file(GLOB SRC
  "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/*.cxx"
  "${CMAKE_CURRENT_SOURCE_DIR}/*.cc"
  "${CMAKE_CURRENT_SOURCE_DIR}/*.c")

make_gettext_mofiles(${target} mofiles)

add_cnoid_plugin(${target} SHARED ${INC} ${SRC} ${mofiles})

target_link_libraries(${target} CnoidBase)

apply_common_setting_for_plugin(${target})

#configure_file(TrafficControlSample.cnoid ${CNOID_SOURCE_SHARE_DIR}/project COPYONLY)
