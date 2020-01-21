set(target PyMulticopterPlugin)
choreonoid_add_python_module(${target} PyMulticopterPlugin.cpp)
target_link_libraries(${target} CnoidMulticopterPlugin CnoidPyBase)
