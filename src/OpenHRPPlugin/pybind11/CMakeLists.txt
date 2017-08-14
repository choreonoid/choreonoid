set(target PyOpenHRP31Plugin)
add_cnoid_python_module(${target} PyItems.cpp)
target_link_libraries(${target} CnoidOpenHRP3.1Plugin CnoidPyBase)
