
# @author Shin'ichiro Nakaoka

set(target PyBodyPlugin)

add_cnoid_python_module(${target}
  PyBodyPluginModule.cpp
  PyItems.cpp
)

target_link_libraries(${target} CnoidBodyPlugin CnoidPyBase)
