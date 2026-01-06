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
  )

set(headers
  FountainDevice.h
  FireDevice.h
  )

choreonoid_qt_add_resources(RC_SRCS SceneEffectsPlugin.qrc)
set(target CnoidSceneEffectsPlugin)
make_gettext_mofiles(${target} mofiles)
choreonoid_add_plugin(${target} ${sources} ${mofiles} ${RC_SRCS} HEADERS ${headers})
target_link_libraries(${target} PUBLIC CnoidBodyPlugin PRIVATE CnoidOpenGL)
