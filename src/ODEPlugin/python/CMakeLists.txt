
# @author Hisashi Ikari

set(target PyODEPlugin)
add_cnoid_python_module(${target} PyODEPlugin.cpp)
target_link_libraries(${target} CnoidODEPlugin CnoidPyBase ${PYTHON_LIBRARIES} ${Boost_PYTHON_LIBRARY})
