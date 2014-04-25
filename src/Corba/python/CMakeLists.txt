
# @author Shin'ichiro Nakaoka

set(target CnoidPyCorba)
add_cnoid_python_module(${target} PyCorba.cpp)
target_link_libraries(${target} CnoidCorba ${PYTHON_LIBRARIES} ${Boost_PYTHON_LIBRARY})
configure_file(omniORBpy.h ${PROJECT_SOURCE_DIR}/include/cnoid COPYONLY)
