set(target PyCorba)
add_cnoid_python_module(${target} PyCorba.cpp)
target_link_libraries(${target} CnoidCorba CnoidPython)
configure_file(omniORBpy.h ${PROJECT_SOURCE_DIR}/include/cnoid COPYONLY)
