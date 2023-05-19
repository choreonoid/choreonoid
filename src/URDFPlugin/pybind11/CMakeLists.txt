set(target PyURDFPlugin)

choreonoid_add_python_module(${target}
  PyURDFPluginModule.cpp
)

target_link_libraries(${target} CnoidURDFPlugin CnoidURDFBodyWriter CnoidURDFBodyLoader)
