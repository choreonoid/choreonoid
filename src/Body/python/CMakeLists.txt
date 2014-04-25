
set(target CnoidPyBody)
add_cnoid_python_module(${target} PyBody.cpp)
target_link_libraries(${target} CnoidBody ${PYTHON_LIBRARIES} ${Boost_PYTHON_LIBRARY})
