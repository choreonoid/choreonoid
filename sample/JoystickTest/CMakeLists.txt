if(NOT ENABLE_GUI OR WIN32 OR APPLE)
  return()
endif()

option(BUILD_JOYSTICK_TEST_PLUGIN "Building a joystick test plugin" OFF)
mark_as_advanced(BUILD_JOYSTICK_TEST_PLUGIN)

if(BUILD_JOYSTICK_TEST_PLUGIN)
  set(target CnoidJoystickTestPlugin)
  choreonoid_add_plugin(${target} SHARED JoystickTestPlugin.cpp)
endif()
