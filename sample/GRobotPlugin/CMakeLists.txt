# @author Shin'ichiro Nakaoka

option(BUILD_GROBOT_PLUGIN "Building GRobotPlugin" OFF)

if(NOT BUILD_GROBOT_PLUGIN)
  return()
endif()

# set(CMAKE_BUILD_TYPE Debug)

set(target CnoidGRobotPlugin)

make_gettext_mofiles(${target} mofiles)

QT5_ADD_RESOURCES(RC_SRCS GRobotPlugin.qrc)

add_cnoid_plugin(${target} SHARED
  GRobotPlugin.cpp
  GRobotBar.cpp
  GRobotControllerItem.cpp
  GRobotController.cpp
  ${mofiles}
  ${RC_SRCS}
  )

if(UNIX)
  target_link_libraries(${target} CnoidBodyPlugin ${Boost_THREAD_LIBRARY} pthread rt)
elseif(MSVC)
  target_link_libraries(${target} CnoidBodyPlugin ${Boost_THREAD_LIBRARY} ${Boost_DATE_TIME_LIBRARY})
  set_target_properties(${target} PROPERTIES COMPILE_DEFINITIONS "BOOST_ASIO_DISABLE_STD_ARRAY")
endif()

apply_common_setting_for_plugin(${target})

install_external_libraries(${Boost_LIBRARY_DIRS} ${Boost_LIBRARY_DIRS} ${Boost_THREAD_LIBRARY} ${Boost_DATE_TIME_LIBRARY})

add_cnoid_body_handler(GRobotHandler GRobotHandler.cpp)

# Deprecated
# add_cnoid_body_customizer(GRobotCustomizer deprecated/GRobotCustomizer.cpp)
