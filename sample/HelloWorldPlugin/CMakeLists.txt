
if(NOT ENABLE_GUI)
  return()
endif()

option(BUILD_HELLO_WORLD_SAMPLE "Building a Hello World sample plugin" OFF)

if(BUILD_HELLO_WORLD_SAMPLE)
  set(target CnoidHelloWorldPlugin)
  add_cnoid_plugin(${target} SHARED HelloWorldPlugin.cpp)
  target_link_libraries(${target} CnoidBase)
  apply_common_setting_for_plugin(${target})

  if(QT5)
    qt5_use_modules(${target} Widgets)
  endif()
endif()

install_sample_source(HelloWorldPlugin.cpp)
