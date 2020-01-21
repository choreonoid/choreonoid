set(target PyAGXDynamicsPlugin)
choreonoid_add_python_module(${target} PyAGXDynamicsPlugin.cpp)
target_link_libraries(${target} CnoidAGXDynamicsPlugin CnoidPyBase)
