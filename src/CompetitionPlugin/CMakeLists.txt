option(BUILD_COMPETITION_PLUGIN "Building CompetitionPlugin" OFF)
if(NOT BUILD_COMPETITION_PLUGIN)
  return()
endif()

set(sources
  CompetitionPlugin.cpp
  CompetitorMarker.cpp
)

set(target CnoidCompetitionPlugin)
add_cnoid_plugin(${target} SHARED ${sources})
target_link_libraries(${target} CnoidBodyPlugin)
apply_common_setting_for_plugin(${target})
