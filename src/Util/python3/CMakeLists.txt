
# @author Shin'ichiro Nakaoka

if(NOT ENABLE_PYTHON)
  return()
endif()

#set_source_files_properties(PyTask.cpp PROPERTIES COMPILE_FLAGS "-O0 -g")

add_cnoid_python_module(PyUtil
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

target_link_libraries(PyUtil CnoidUtil CnoidPython)

set(headers
  PyUtil.h
  PySignal.h
  )

apply_common_setting_for_python_module(PyUtil "${headers}")
