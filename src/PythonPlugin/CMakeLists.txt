if(NOT ENABLE_PYTHON OR NOT ENABLE_GUI)
  return()
endif()

choreonoid_add_python_binding_subdirectory()
