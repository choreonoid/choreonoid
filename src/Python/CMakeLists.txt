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

include_directories(${PYTHON_INCLUDE_PATH})
add_cnoid_library(${target} SHARED ${sources} ${headers})
target_link_libraries(${target} ${PYTHON_LIBRARIES} ${Boost_PYTHON_LIBRARY})
apply_common_setting_for_library(${target} "${headers}")
