set(headers
  PyUtil.h
  PyReferenced.h
  PySignal.h
  PyEigenTypes.h
  )

choreonoid_add_library(CnoidPyUtil SHARED PyUtil.cpp HEADERS ${headers})
target_link_libraries(CnoidPyUtil PUBLIC ${PYTHON_LIBRARIES})

choreonoid_add_python_module(PyUtil
  PyUtilModule.cpp
  PyReferenced.cpp
  PyMessageOut.cpp
  PySignal.cpp
  PyValueTree.cpp
  PyEigenTypes.cpp
  PyEigenArchive.cpp
  PyFilePathVariableProcessor.cpp
  PySeqTypes.cpp
  PySceneGraph.cpp
  PySceneDrawables.cpp
  PySceneRenderer.cpp
  PyMeshUtils.cpp
  PyGeometryTypes.cpp
  PyTask.cpp
  )

target_link_libraries(PyUtil CnoidUtil)
