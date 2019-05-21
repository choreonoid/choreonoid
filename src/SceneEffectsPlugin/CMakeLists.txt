option(BUILD_SCENE_EFFECTS_PLUGIN "Building SceneEffectsPlugin" OFF)

if(NOT BUILD_SCENE_EFFECTS_PLUGIN)
  return()
endif()

set(sources
  SceneEffectsPlugin.cpp
  ParticleSystem.cpp
  SceneParticles.cpp
  SceneFountain.cpp
  SceneFire.cpp
  SceneSmoke.cpp
  SceneRainSnow.cpp
  ParticlesProgram.cpp
  FountainDevice.cpp
  SmokeDevice.cpp
  FireDevice.cpp
  RainSnowDevice.cpp
  gl_core_3_3.c
  )

set(headers )

QT5_ADD_RESOURCES(RC_SRCS SceneEffectsPlugin.qrc)
set(target CnoidSceneEffectsPlugin)
make_gettext_mofiles(${target} mofiles)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
add_cnoid_plugin(${target} SHARED ${sources} ${headers} ${mofiles} ${RC_SRCS})
target_link_libraries(${target} CnoidBodyPlugin)
apply_common_setting_for_plugin(${target} "${headers}")
