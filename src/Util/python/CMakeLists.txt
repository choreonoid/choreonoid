# Note: the nanobind backend does not create a CnoidPyUtil shared library
# (unlike the pybind11 backend). PyUtil.h / PyReferenced.h are shared headers
# but they need no companion runtime library:
#   - <nanobind/nanobind.h> is reachable through the nanobind core library that
#     each Python module already links to via nanobind_add_module().
#   - <cnoid/PyUtil> / <cnoid/PyReferenced> shims (and their src-relative includes)
#     resolve through the top-level include_directories(${PROJECT_SOURCE_DIR}).
#   - CnoidUtil and Python are linked directly by each module that needs them.
# choreonoid_add_python_module() therefore does not link to CnoidPyUtil in
# nanobind mode; ChoreonoidConfig.cmake.in likewise skips the import target.

# Install the shared Python binding headers for SDK consumers. <cnoid/PyUtil>
# and <cnoid/PyReferenced> shims in include/cnoid/ resolve to these files via
# their relative #include "src/Util/python/...".
choreonoid_set_header_files(INSTALL_HEADERS HEADERS
  PyUtil.h
  PyReferenced.h
  PySignal.h
  PyEigenTypes.h
  PyNumpyHelper.h
  )

choreonoid_add_python_module(PyUtil
  PyUtilModule.cpp
  PyReferenced.cpp
  PySignal.cpp
  PyMessageOut.cpp
  PyValueTree.cpp
  PyEigenTypes.cpp
  PyEigenArchive.cpp
  PyFilePathVariableProcessor.cpp
  PySeqTypes.cpp
  PyGeometryTypes.cpp
  PySceneGraph.cpp
  PySceneDrawables.cpp
  PyMeshUtils.cpp
  PySceneRenderer.cpp
  PyTask.cpp
  )

target_link_libraries(PyUtil PRIVATE CnoidUtil)
