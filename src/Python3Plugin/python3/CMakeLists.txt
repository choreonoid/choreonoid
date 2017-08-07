
# @author Shin'ichiro Nakaoka

set(target PyPythonPlugin)
add_cnoid_python3_module(${target} PyPythonPlugin.cpp)
target_link_libraries(${target} CnoidPython3Plugin)
