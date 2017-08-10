set(target PyPythonPlugin)
add_cnoid_python_module(${target} PyPythonPlugin.cpp)
target_link_libraries(${target} CnoidPythonPlugin)
