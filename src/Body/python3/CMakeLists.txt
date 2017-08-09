if(NOT ENABLE_PYTHON3)
  return()
endif()

add_cnoid_python3_module(Py3Body PyBodyModule.cpp)
target_link_libraries(Py3Body CnoidBody CnoidPython3)
