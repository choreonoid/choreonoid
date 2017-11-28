set(target PyOpenRTMPlugin)
add_cnoid_python_module(${target} PyOpenRTMPlugin.cpp)
target_link_libraries(${target} CnoidOpenRTMPlugin CnoidPyBase)
