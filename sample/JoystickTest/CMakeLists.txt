
if(NOT ENABLE_GUI OR WIN32 OR APPLE)
  return()
endif()

option(BUILD_JOYSTICK_TEST_PLUGIN "Building a joystick test plugin" OFF)

if(BUILD_JOYSTICK_TEST_PLUGIN)
  set(target CnoidJoystickTestPlugin)
  add_cnoid_plugin(${target} SHARED JoystickTestPlugin.cpp)
  target_link_libraries(${target} CnoidBase)
  apply_common_setting_for_plugin(${target})
endif()
