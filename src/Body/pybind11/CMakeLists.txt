add_cnoid_python_module(PyBody
  PyBodyModule.cpp
  PyBody.cpp
  PyLink.cpp
  PyDeviceTypes.cpp
  )

target_link_libraries(PyBody CnoidBody CnoidPyUtil)
apply_common_setting_for_python_module(PyBody)