choreonoid_add_python_module(PyBodyPlugin
  PyBodyPluginModule.cpp
  PyBodyItem.cpp
  PyItems.cpp
  PySimulationClasses.cpp
  )

target_link_libraries(PyBodyPlugin PRIVATE CnoidBodyPlugin CnoidPyBase)
