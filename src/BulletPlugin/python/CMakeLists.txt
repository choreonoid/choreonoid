
# @author Hisashi Ikari

set(target PyBulletPlugin)
add_cnoid_python_module(${target} PyBulletPlugin.cpp)
target_link_libraries(${target} CnoidBulletPlugin ${PYTHON_LIBRARIES} ${Boost_PYTHON_LIBRARY})
