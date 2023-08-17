set(target PyAssimpPlugin)

choreonoid_add_python_module(${target}
  PyAssimpPluginModule.cpp
)

target_link_libraries(${target} CnoidAssimpPlugin CnoidAssimpSceneLoader CnoidAssimpSceneWriter)
