if(NOT ENABLE_URDF OR NOT ENABLE_GUI)
  return()
endif()

make_gettext_mofiles(CnoidURDFPlugin mofiles)
choreonoid_add_plugin(CnoidURDFPlugin URDFPlugin.cpp ${mofiles})
target_link_libraries(CnoidURDFPlugin PUBLIC CnoidBodyPlugin CnoidURDFBodyLoader)
