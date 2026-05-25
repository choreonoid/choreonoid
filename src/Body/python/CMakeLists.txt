choreonoid_add_python_module(PyBody
  PyBodyModule.cpp
  PyBody.cpp
  PyLink.cpp
  PyDeviceTypes.cpp
  PyMaterial.cpp
  )

target_link_libraries(PyBody PRIVATE CnoidBody)
