set(target CnoidGLGearsPlugin)
set(sources GLGearsPlugin.cpp GLGearsView.cpp)

if(CMAKE_PROJECT_NAME STREQUAL "Choreonoid")
  # Build inside the Choreonoid project
  choreonoid_add_plugin(${target} ${sources})
  target_link_libraries(${target} CnoidBase)
else()
  # Stand alone build
  cmake_minimum_required(VERSION 3.1.0)
  project(GLGearsPlugin)
  find_package(Choreonoid REQUIRED)
  add_definitions(${CHOREONOID_DEFINITIONS})
  include_directories(${CHOREONOID_INCLUDE_DIRS})
  link_directories(${CHOREONOID_LIBRARY_DIRS})
  add_library(${target} SHARED ${sources})
  target_link_libraries(${target} ${CHOREONOID_BASE_LIBRARIES})
  install(TARGETS ${target} LIBRARY DESTINATION ${CHOREONOID_PLUGIN_DIR})
endif()
