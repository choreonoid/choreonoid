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
target_link_libraries(${target} ${OMNIDYNAMIC_LIBRARIES} fmt::fmt)
apply_common_setting_for_library(${target} "${headers}")

# name server
set(target cnoid-nameserver)
idl_compile_cpp(idl_cpp_files idl_h_files corba Naming LOCAL)
add_cnoid_executable(${target} NameServer.cpp CosNaming_impl.cpp ${idl_cpp_files})
target_link_libraries(${target} ${OMNIDYNAMIC_LIBRARIES})

if(UNIX)
  configure_file(reset-omninames.sh ${PROJECT_BINARY_DIR}/bin/reset-omninames.sh COPYONLY)
endif()
