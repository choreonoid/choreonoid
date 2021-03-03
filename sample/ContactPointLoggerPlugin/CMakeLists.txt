if(NOT ENABLE_GUI)
  return()
endif()

option(BUILD_CONTACT_POINT_LOGGER_PLUGIN "Building ContactPointLoggerPlugin" OFF)
if(NOT BUILD_CONTACT_POINT_LOGGER_PLUGIN)
  return()
endif()

set(target CnoidContactPointLoggerPlugin)
choreonoid_add_plugin(${target} SHARED ContactPointLoggerPlugin.cpp)
target_link_libraries(${target} CnoidBodyPlugin)

install(FILES SR1Liftup-ContactPointLog.cnoid DESTINATION ${CNOID_SHARE_SUBDIR}/project)
