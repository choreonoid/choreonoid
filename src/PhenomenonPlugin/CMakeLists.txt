
# @author Shin'ichiro Nakaoka

option(BUILD_PHENOMENON_PLUGIN "Building PhenomenonPlugin" OFF)

if(NOT BUILD_PHENOMENON_PLUGIN)
  return()
endif()

set(sources
  PhenomenonPlugin.cpp
  SceneParticles.cpp
  SceneFountain.cpp
  SceneRainSnow.cpp
  ParticlesProgram.cpp
  FountainDevice.cpp
  RainSnowDevice.cpp
  gl_core_3_3.c
  )

set(headers

  )

if(NOT QT5)
  QT4_ADD_RESOURCES(RC_SRCS PhenomenonPlugin.qrc)
else()
  QT5_ADD_RESOURCES(RC_SRCS PhenomenonPlugin.qrc)
endif()

set(target CnoidPhenomenonPlugin)

make_gettext_mofiles(${target} mofiles)

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

add_cnoid_plugin(${target} SHARED ${sources} ${headers} ${mofiles} ${RC_SRCS})

target_link_libraries(${target} CnoidBase CnoidBodyPlugin)
apply_common_setting_for_plugin(${target} "${headers}")
