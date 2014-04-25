
# @author Shin'ichiro Nakaoka

#set(CMAKE_BUILD_TYPE Debug)

if(NOT ENABLE_CORBA)
  return()
endif()

set(target CnoidCorba)

set(sources
  CorbaUtil.cpp	
  )

set(headers
  CorbaUtil.h
  )

add_cnoid_library(${target} SHARED ${sources} ${headers})
target_link_libraries(${target} ${OMNIORB_LIBRARIES} ${Boost_THREAD_LIBRARY}) # omniDynamic4 

apply_common_setting_for_library(${target} "${headers}")


if(ENABLE_PYTHON)
  add_subdirectory(python)
endif()
