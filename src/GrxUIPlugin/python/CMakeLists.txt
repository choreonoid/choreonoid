set(target Pygrxui)
add_cnoid_python_module(${target} PyGrxUI.cpp)
target_link_libraries(${target} CnoidGrxUIPlugin CnoidPyBase)
