if(NOT ENABLE_SDF OR NOT ENABLE_GUI)
  return()
endif()

make_gettext_mofiles(CnoidSDFPlugin mofiles)
choreonoid_add_plugin(CnoidSDFPlugin SDFPlugin.cpp ${mofiles})
target_link_libraries(CnoidSDFPlugin PUBLIC CnoidBodyPlugin CnoidSDFBodyLoader)
