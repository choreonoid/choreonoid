
# @author Hisashi Ikari

set(target CnoidPyODEPlugin)
add_cnoid_python_module(${target} PyODEPlugin.cpp)
target_link_libraries(${target} CnoidODEPlugin ${PYTHON_LIBRARIES} ${Boost_PYTHON_LIBRARY})
