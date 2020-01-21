set(target PyPythonSimScriptPlugin)
choreonoid_add_python_module(${target} PyPythonSimScriptPlugin.cpp)
target_link_libraries(${target} CnoidPythonSimScriptPlugin CnoidPyBase)
