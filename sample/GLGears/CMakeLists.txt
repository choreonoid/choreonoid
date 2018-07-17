if(NOT ENABLE_GUI)
  return()
endif()

option(BUILD_GL_GEARS_SAMPLE "Building a GL Gears sample plugin" OFF)
set(sources GLGearsPlugin.cpp GLGearsView.cpp GLGearsView.h)

if(BUILD_GL_GEARS_SAMPLE)
  set(target CnoidGLGearsPlugin)
  add_cnoid_plugin(${target} SHARED ${sources})
  target_link_libraries(${target} CnoidBase)
  apply_common_setting_for_plugin(${target})
endif()

#install_sample_source(${sources})
