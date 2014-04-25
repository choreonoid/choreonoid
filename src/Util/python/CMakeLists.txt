
# @author Shin'ichiro Nakaoka

set(target CnoidPyUtil)
add_cnoid_python_module(${target} PyUtil.cpp)
target_link_libraries(${target} CnoidUtil ${PYTHON_LIBRARIES} ${Boost_PYTHON_LIBRARY})
