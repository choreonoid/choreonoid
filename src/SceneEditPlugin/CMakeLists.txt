
option(BUILD_SCENEEDIT_PLUGIN "Building Scene Edit Plugin" OFF)

if(NOT BUILD_SCENEEDIT_PLUGIN)
  return()
endif()

set(sources
    SceneEditPlugin.cpp
    SceneGraphView.cpp
    SceneGraphPropertyView.cpp
  )

set(headers
  SceneGraphView.h
  SceneGraphPropertyView.h
)

set(target CnoidSceneEditPlugin)

make_gettext_mofiles(${target} mofiles)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
add_cnoid_plugin(${target} SHARED ${sources} ${headers} ${mofiles} )
target_link_libraries(${target} CnoidUtil CnoidBase )
apply_common_setting_for_plugin(${target} "${headers}")
