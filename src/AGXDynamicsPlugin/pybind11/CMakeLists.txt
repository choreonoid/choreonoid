set(target PyAGXDynamicsPlugin)
add_cnoid_python_module(${target} PyAGXDynamicsPlugin.cpp)
target_link_libraries(${target} CnoidAGXDynamicsPlugin CnoidPyBase)
