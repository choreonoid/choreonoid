# CnoidPythonInterpreter provides the lifecycle management (initialization and
# finalization) of the embedded CPython interpreter for the nanobind backend.
# It is built only for the nanobind backend (see the guard in src/CMakeLists.txt)
# and is loaded dynamically by the Base App at startup so that the Base module
# does not have a compile-time dependency on Python.

set(target CnoidPythonInterpreter)

choreonoid_add_library(${target} SHARED PythonInterpreter.cpp HEADERS PythonInterpreter.h)
choreonoid_make_header_public(PythonInterpreter.h)
target_link_libraries(${target} PUBLIC CnoidUtil ${PYTHON_LIBRARIES})
