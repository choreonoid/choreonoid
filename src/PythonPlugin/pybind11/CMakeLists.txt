set(target PyPythonPlugin)
choreonoid_add_python_module(${target} PyPythonPlugin.cpp)
target_link_libraries(${target} CnoidPythonPlugin CnoidPyBase)
