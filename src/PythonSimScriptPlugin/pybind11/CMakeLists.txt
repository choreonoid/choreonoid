set(target PyPythonSimScriptPlugin)
add_cnoid_python_module(${target} PyPythonSimScriptPlugin.cpp)
target_link_libraries(${target} CnoidPythonSimScriptPlugin CnoidPyBase)
