if(NOT ENABLE_PYTHON3)
  return()
endif()

add_cnoid_python3_module(PyUtil
  PyUtilModule.cpp
  PySignal.cpp
  PyValueTree.cpp
  PyEigenTypes.cpp
  PyEigenArchive.cpp
  PySeqTypes.cpp
  PySceneGraph.cpp
  PyGeometryTypes.cpp
  PyTask.cpp
  )

target_link_libraries(PyUtil CnoidUtil CnoidPython3)

set(headers
  PyUtil.h
  PySignal.h
  )

apply_common_setting_for_python_module(PyUtil "${headers}")
