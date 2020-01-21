option(BUILD_COMPETITION_PLUGIN "Building CompetitionPlugin" OFF)
if(NOT BUILD_COMPETITION_PLUGIN)
  return()
endif()

set(sources
  CompetitionPlugin.cpp
  CompetitorMarker.cpp
)

set(target CnoidCompetitionPlugin)
choreonoid_add_plugin(${target} ${sources})
target_link_libraries(${target} CnoidBodyPlugin)
