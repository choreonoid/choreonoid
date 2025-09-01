option(BUILD_LIVOX_MID360_PLUGIN "Building LivoxMid360 plugin" OFF)
if(NOT BUILD_LIVOX_MID360_PLUGIN)
  return()
elseif(NOT BUILD_GL_VISION_SIMULATOR_PLUGIN)
  message(FATAL_ERROR "The LivoxMid360 plugin requires GLVisionSimulatorPlugin, which is enabled by the BUILD_GL_VISION_SIMULATOR_PLUGIN option")
endif()

set(target CnoidLivoxMid360Plugin)

set(sources
  LivoxMid360Plugin.cpp
  LivoxMid360.cpp
  GLLivoxMid360Simulator.cpp
  )

set(headers
  LivoxMid360.h
  )

choreonoid_make_headers_public(${headers})

choreonoid_add_plugin(${target} ${sources})
target_link_libraries(${target} PRIVATE CnoidGLVisionSimulatorPlugin)
