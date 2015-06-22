
# @author Shin'ichiro Nakaoka

#set_source_files_properties(PyTask.cpp PROPERTIES COMPILE_FLAGS "-O0 -g")

add_cnoid_python_module(PyUtil
  PyUtilModule.cpp
  PySignal.cpp
  PyValueTree.cpp
  PyEigenTypes.cpp
  PyEigenArchive.cpp
  PySeqTypes.cpp
  PySceneGraph.cpp
  PyTask.cpp
  )

target_link_libraries(PyUtil CnoidUtil CnoidPython)
