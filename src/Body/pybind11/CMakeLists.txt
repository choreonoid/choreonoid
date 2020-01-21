choreonoid_add_python_module(PyBody
  PyBodyModule.cpp
  PyBody.cpp
  PyLink.cpp
  PyDeviceTypes.cpp
  )

target_link_libraries(PyBody CnoidBody CnoidPyUtil)
