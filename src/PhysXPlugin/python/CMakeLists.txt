set(target PyPhysXPlugin)
choreonoid_add_python_module(${target} PyPhysXPlugin.cpp)
target_link_libraries(${target} PRIVATE CnoidPhysXPlugin CnoidPyBase)
