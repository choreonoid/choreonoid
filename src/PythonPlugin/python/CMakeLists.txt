
# @author Shin'ichiro Nakaoka

set(target CnoidPyPython)
add_cnoid_python_module(${target} PyPython.cpp)
target_link_libraries(${target} CnoidPythonPlugin)
