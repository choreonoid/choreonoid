
#set(CMAKE_BUILD_TYPE Debug)

if(NOT ENABLE_GUI)
  return()
endif()

option(BUILD_SUBMERSIBLE "Building a subumersible" OFF)
if(NOT BUILD_SUBMERSIBLE)
  return()
endif()

set(sources
  SubmersiblePlugin.cpp
  ThrusterDevice.cpp
  SubmersibleSimulatorItem2.cpp
)

set(header
  ThrusterDevice.h
  SubmersibleSimulatorItem2.h
)

set(target CnoidSubmersiblePlugin)
add_cnoid_plugin(${target} SHARED ${sources})
add_cnoid_simple_controller(SubmersibleController SubmersibleController.cpp)
target_link_libraries(${target} CnoidBodyPlugin)
apply_common_setting_for_plugin(${target})

#configure_file(submersible.body ${CNOID_SOURCE_SHARE_DIR}/model/misc COPYONLY)
#configure_file(FogUnderWater.wrl ${CNOID_SOURCE_SHARE_DIR}/model/misc COPYONLY)
#configure_file(SubmersibleSample.cnoid ${CNOID_SOURCE_SHARE_DIR}/project COPYONLY)
              
