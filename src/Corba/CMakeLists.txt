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
  exportdecl.h
  )

choreonoid_add_library(${target} SHARED ${sources} HEADERS ${headers})
target_link_libraries(${target} ${CHOREONOID_OMNIORB_LIBRARIES} fmt::fmt)

# name server
set(target cnoid-nameserver)
choreonoid_compile_corba_idl_to_cpp(idl_cpp_files idl_h_files corba Naming LOCAL)
choreonoid_add_executable(${target} NameServer.cpp CosNaming_impl.cpp ${idl_cpp_files})
target_link_libraries(${target} ${CHOREONOID_OMNIORB_LIBRARIES})

if(UNIX)
  configure_file(reset-omninames.sh ${PROJECT_BINARY_DIR}/bin/reset-omninames.sh COPYONLY)
  install(PROGRAMS reset-omninames.sh DESTINATION bin)
endif()
