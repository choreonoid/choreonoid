
# @author Shin'ichiro Nakaoka

option(BUILD_ROBOT_ACCESS_PLUGIN "Building the RobotAccess Plugin" OFF)
if(NOT BUILD_ROBOT_ACCESS_PLUGIN)
  return()
endif()

#set(CMAKE_BUILD_TYPE Debug)

set(target CnoidRobotAccessPlugin)

set(sources
  RobotAccessPlugin.cpp
  RobotAccessItem.cpp
  RobotAccessBar.cpp
  )

set(headers
  RobotAccessItem.h
  )

choreonoid_make_gettext_mo_files(${target} mofiles)

qt5_add_resources(RC_SRCS RobotAccessPlugin.qrc)

choreonoid_add_plugin(${target} ${sources} ${mofiles} ${RC_SRCS} HEADERS ${headers})
target_link_libraries(${target} CnoidBodyPlugin)
