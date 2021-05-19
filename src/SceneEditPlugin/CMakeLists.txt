option(BUILD_SCENEEDIT_PLUGIN "Building Scene Edit Plugin" OFF)
mark_as_advanced(BUILD_SCENEEDIT_PLUGIN)

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

choreonoid_make_gettext_mo_files(${target} mofiles)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
choreonoid_add_plugin(${target} ${sources} ${mofiles} HEADERS ${headers})
