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
choreonoid_add_plugin(${target} ${sources})
target_link_libraries(${target} PUBLIC CnoidBodyPlugin)
install_project_files(SubmersibleSample.cnoid)
