
# @author Shin'ichiro Nakaoka

set(target CnoidPyUtil)

add_cnoid_python_module(${target}
  PyUtil.cpp
  PySignal.cpp
  PyEigenTypes.cpp
  PySeqTypes.cpp
  PySceneGraph.cpp
  )

target_link_libraries(${target} CnoidUtil CnoidPython)
