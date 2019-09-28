
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

make_gettext_mofiles(${target} mofiles)

QT5_ADD_RESOURCES(RC_SRCS RobotAccessPlugin.qrc)

add_cnoid_plugin(${target} SHARED ${sources} ${headers} ${mofiles} ${RC_SRCS})
target_link_libraries(${target} CnoidBodyPlugin)
apply_common_setting_for_plugin(${target} "${headers}")
