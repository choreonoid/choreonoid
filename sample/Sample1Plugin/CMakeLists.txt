
if(NOT ENABLE_GUI)
  return()
endif()

option(BUILD_SAMPLE1_SAMPLE "Building a sample plugin \"Sample1Plugin\"" OFF)

if(BUILD_SAMPLE1_SAMPLE)
  set(target CnoidSample1Plugin)
  add_cnoid_plugin(${target} SHARED Sample1Plugin.cpp)
  target_link_libraries(${target} CnoidBodyPlugin)
  apply_common_setting_for_plugin(${target})

  if(QT5)
    qt5_use_modules(${target} Widgets)
  endif()
endif()

install_sample_source(Sample1Plugin.cpp)
