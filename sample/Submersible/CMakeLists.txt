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

install(FILES SubmersibleSample.cnoid DESTINATION ${CNOID_SHARE_SUBDIR}/project)
