
# @author Shin'ichiro Nakaoka
# @author Hisashi Ikari

set(target CnoidPySimpleControllerPlugin)
add_cnoid_python_module(${target} PySimpleControllerPlugin.cpp)
target_link_libraries(${target} CnoidSimpleControllerPlugin ${PYTHON_LIBRARIES} ${Boost_PYTHON_LIBRARY})
