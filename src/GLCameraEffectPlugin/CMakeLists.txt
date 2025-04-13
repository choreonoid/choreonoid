if(NOT BUILD_GL_VISION_SIMULATOR_PLUGIN)
  return()
endif()

option(BUILD_GL_CAMERA_EFFECT_PLUGIN "Building GLCameraEffectPlugin" OFF)

if(NOT BUILD_GL_CAMERA_EFFECT_PLUGIN)
  return()
endif()

set(target CnoidGLCameraEffectPlugin)

set(sources
  GLCameraEffectPlugin.cpp
  GLCameraEffectSimulator.cpp
  GLRangeCameraEffectSimulator.cpp
  ImageFilter.cpp
)

set(headers
  GLCameraEffectSimulator.h
  GLRangeCameraEffectSimulator.h
  ImageFilter.h
  exportdecl.h
)

choreonoid_make_gettext_mo_files(${target} mofiles)
choreonoid_add_plugin(${target} ${sources} ${mofiles} HEADERS ${headers})
target_link_libraries(${target} PUBLIC CnoidGLVisionSimulatorPlugin)