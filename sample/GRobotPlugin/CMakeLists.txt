# @author Shin'ichiro Nakaoka

option(BUILD_GROBOT_PLUGIN "Building GRobotPlugin" OFF)

if(NOT BUILD_GROBOT_PLUGIN)
  return()
endif()

# set(CMAKE_BUILD_TYPE Debug)

set(target CnoidGRobotPlugin)

choreonoid_make_gettext_mo_files(${target} mofiles)

qt5_add_resources(RC_SRCS GRobotPlugin.qrc)

choreonoid_add_plugin(${target}
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

choreonoid_add_body_handler(GRobotHandler GRobotHandler.cpp)

# Deprecated
# choreonoid_add_body_customizer(GRobotCustomizer deprecated/GRobotCustomizer.cpp)
