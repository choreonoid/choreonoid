
# @author Hisashi Ikari

set(target CnoidPyPythonSimScriptPlugin)
add_cnoid_python_module(${target} PyPythonSimScriptPlugin.cpp)
target_link_libraries(${target} CnoidPythonSimScriptPlugin)
