set(target PyMulticopterPlugin)
add_cnoid_python_module(${target} PyMulticopterPlugin.cpp)
target_link_libraries(${target} CnoidMulticopterPlugin CnoidPyBase)
