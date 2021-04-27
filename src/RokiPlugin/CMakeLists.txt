option(BUILD_ROKI_PLUGIN "Building RokiPlugin" OFF)
mark_as_advanced(BUILD_ROKI_PLUGIN)

if(NOT BUILD_ROKI_PLUGIN)
  return()
endif()

if(NOT UNIX)
  return()
endif()

set(ROKI_DIR ${ROKI_DIR} CACHE PATH "set the top directory of Roki libraries")
set(ROKI_INCLUDE_DIRS ${ROKI_DIR}/include)
set(ROKI_LIBRARY_DIRS ${ROKI_DIR}/lib)
include_directories(${ROKI_INCLUDE_DIRS})
link_directories(${ROKI_LIBRARY_DIRS})
set(ROKI_LIBRARIES cure zm zeo roki)

set(target CnoidRokiPlugin)

set(sources
  RokiPlugin.cpp
  RokiSimulatorItem.cpp
)

set(headers
)

make_gettext_mofiles(${target} mofiles) 
choreonoid_add_plugin(${target} ${sources} ${mofiles} HEADERS ${headers})
target_link_libraries(${target} CnoidBodyPlugin ${ROKI_LIBRARIES})
install_runtime_dlls(${ROKI_DIR}/bin ${ROKI_LIBRARIES})
