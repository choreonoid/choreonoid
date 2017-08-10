set(target PyMedia)
add_cnoid_python_module(${target} PyMedia.cpp)
target_link_libraries(${target} CnoidMediaPlugin CnoidPyBase)
