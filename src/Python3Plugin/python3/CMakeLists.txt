set(target Py3Python3Plugin)
add_cnoid_python3_module(${target} PyPython3Plugin.cpp)
target_link_libraries(${target} CnoidPython3Plugin)
