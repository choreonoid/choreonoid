
# @author Shin'ichiro Nakaoka
# @author Hisashi Ikari

set(target CnoidPyPoseSeqPlugin)
add_cnoid_python_module(${target} PyPoseSeqPlugin.cpp)
target_link_libraries(${target} CnoidPoseSeqPlugin ${PYTHON_LIBRARIES} ${Boost_PYTHON_LIBRARY})
