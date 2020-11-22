set(target PyMediaPlugin)
choreonoid_add_python_module(${target} PyMediaPlugin.cpp)
target_link_libraries(${target} CnoidMediaPlugin CnoidPyBase)
