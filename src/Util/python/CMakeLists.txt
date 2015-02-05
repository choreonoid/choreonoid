
# @author Shin'ichiro Nakaoka

add_cnoid_python_module(PyUtil
  PyUtilModule.cpp
  PySignal.cpp
  PyValueTree.cpp
  PyEigenTypes.cpp
  #PyEigenTypes.old.cpp
  PySeqTypes.cpp
  PySceneGraph.cpp
  PyTask.cpp
  )

target_link_libraries(PyUtil CnoidUtil CnoidPython)
