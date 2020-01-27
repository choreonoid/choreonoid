# @author Shin'ichiro Nakaoka

#set(CMAKE_BUILD_TYPE Debug)

option(BUILD_CORBA_PLUGIN "Build CORBA Plugin" OFF)

if(NOT BUILD_CORBA_PLUGIN)
  return()
elseif(NOT ENABLE_CORBA)
  message(FATAL_ERROR "CorbaPlugin needs to ENABLE_CORBA")
endif()

choreonoid_compile_corba_idl_to_cpp(idl_cpp_files idl_h_files corba MessageView)

set(target CnoidCorbaPlugin)

set(sources
  CorbaPlugin.cpp
  NameServerView.cpp
  MessageView_impl.cpp
  )

set(headers
  CorbaPlugin.h
  exportdecl.h
  )

choreonoid_make_gettext_mo_files(${target} mofiles)
choreonoid_add_plugin(${target} ${sources} ${mofiles} ${idl_cpp_files} HEADERS ${headers})
target_link_libraries(${target} CnoidUtil CnoidCorba CnoidBase)

# test program
# add_cnoid_executable(corba-test test.cpp ${idl_cpp_files})
# target_link_libraries(corba-test CnoidCorba ${CHOREONOID_OMNIORB_LIBRARIES})
