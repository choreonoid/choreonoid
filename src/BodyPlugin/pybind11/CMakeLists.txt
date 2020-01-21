set(target PyBodyPlugin)

choreonoid_add_python_module(${target}
  PyBodyPluginModule.cpp
  PyBodyItem.cpp
  PyItems.cpp
  PySimulationClasses.cpp
)

target_link_libraries(${target} CnoidBodyPlugin CnoidPyBase)
