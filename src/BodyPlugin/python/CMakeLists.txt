
# @author Shin'ichiro Nakaoka

if(NOT ENABLE_PYTHON)
  return()
endif()

set(target PyBodyPlugin)

add_cnoid_python_module(${target}
  PyBodyPluginModule.cpp
  PyBodyItem.cpp
  PyItems.cpp
  PySimulationClasses.cpp
)

target_link_libraries(${target} CnoidBodyPlugin CnoidPyBase)
