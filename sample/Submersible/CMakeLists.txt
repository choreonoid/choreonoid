
#set(CMAKE_BUILD_TYPE Debug)

if(NOT ENABLE_GUI)
  return()
endif()

option(BUILD_SUBMERSIBLE_SAMPLE "Building a sabumersible sample" OFF)
if(NOT BUILD_SUBMERSIBLE_SAMPLE)
  return()
endif()

set(sources
  SubmersibleSamplePlugin.cpp
  SubmersibleSimulatorItem.cpp
)

set(target CnoidSubmersibleSamplePlugin)
add_cnoid_plugin(${target} SHARED ${sources})
target_link_libraries(${target} CnoidBodyPlugin)
apply_common_setting_for_plugin(${target})

configure_file(submersible.body ${CNOID_SOURCE_SHARE_DIR}/model/misc COPYONLY)
configure_file(FogUnderWater.wrl ${CNOID_SOURCE_SHARE_DIR}/model/misc COPYONLY)
configure_file(SubmersibleSample.cnoid ${CNOID_SOURCE_SHARE_DIR}/project COPYONLY)
              
