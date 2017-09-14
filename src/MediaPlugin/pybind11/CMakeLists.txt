set(target PyMediaPlugin)
add_cnoid_python_module(${target} PyMediaPlugin.cpp)
target_link_libraries(${target} CnoidMediaPlugin CnoidPyUtil)
