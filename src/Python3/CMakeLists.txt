if(NOT ENABLE_PYTHON3)
  return()
endif()

set(sources
  Python3Util.cpp
  )

set(headers
  Python3Util.h
  )

set(target CnoidPython3)

add_cnoid_library(${target} SHARED ${sources} ${headers})

target_link_libraries(${target} ${PYTHON3_LIBRARIES})

apply_common_setting_for_library(${target} "${headers}")
