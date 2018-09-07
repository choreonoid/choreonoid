set(headers
  PyUtil.h
  PyReferenced.h
  PySignal.h
  PyEigenTypes.h
  )

add_cnoid_library(CnoidPyUtil SHARED PyUtil.cpp ${headers})
target_link_libraries(CnoidPyUtil ${PYTHON_LIBRARIES})
apply_common_setting_for_library(CnoidPyUtil "${headers}")

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

target_link_libraries(PyUtil CnoidUtil CnoidPyUtil)
