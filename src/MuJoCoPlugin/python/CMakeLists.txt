set(target PyMuJoCoPlugin)
choreonoid_add_python_module(${target} PyMuJoCoPlugin.cpp)
target_link_libraries(${target} PRIVATE CnoidMuJoCoPlugin CnoidPyBase)
