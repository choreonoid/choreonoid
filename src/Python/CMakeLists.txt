if(NOT ENABLE_PYTHON)
  return()
endif()

set(sources
  PythonUtil.cpp
  )

set(headers
  PythonUtil.h
  )

set(target CnoidPython)

add_cnoid_library(${target} SHARED ${sources} ${headers})
target_link_libraries(${target} ${PYTHON_LIBRARIES})
apply_common_setting_for_library(${target} "${headers}")
