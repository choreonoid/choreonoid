add_cnoid_python_module(PyUtil
  PyUtilModule.cpp
  PyReferenced.cpp
  PySignal.cpp
  PyValueTree.cpp
  PyEigenTypes.cpp
  PyEigenArchive.cpp
  PySeqTypes.cpp
  PySceneGraph.cpp
  PyGeometryTypes.cpp
  PyTask.cpp
  )

target_link_libraries(PyUtil CnoidUtil CnoidPython)

set(headers
  PyUtil.h
  PyReferenced.h
  PySignal.h
  PyEigenTypes.h
  )

apply_common_setting_for_python_module(PyUtil "${headers}")
