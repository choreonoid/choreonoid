set(target PyPoseSeqPlugin)
choreonoid_add_python_module(${target} PyPoseSeqPlugin.cpp)
target_link_libraries(${target} CnoidPoseSeqPlugin CnoidPyBase)
