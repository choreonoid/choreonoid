
# @author Shin'ichiro Nakaoka

set(target CnoidPyBodyPlugin)

add_cnoid_python_module(${target}
  PyBodyPlugin.cpp
  PyItems.cpp
)

target_link_libraries(${target} CnoidBodyPlugin ${PYTHON_LIBRARIES} ${Boost_PYTHON_LIBRARY})
