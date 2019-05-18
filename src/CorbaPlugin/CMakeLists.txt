# @author Shin'ichiro Nakaoka

#set(CMAKE_BUILD_TYPE Debug)

option(BUILD_CORBA_PLUGIN "Build CORBA Plugin" OFF)

if(NOT BUILD_CORBA_PLUGIN)
  return()
elseif(NOT ENABLE_CORBA)
  message(FATAL_ERROR "CorbaPlugin needs to ENABLE_CORBA")
endif()

idl_compile_cpp(idl_cpp_files idl_h_files corba MessageView)

set(target CnoidCorbaPlugin)

set(sources
  CorbaPlugin.cpp
  NameServerView.cpp
  MessageView_impl.cpp
  )

set(headers
  CorbaPlugin.h
  )

make_gettext_mofiles(${target} mofiles)
add_cnoid_plugin(${target} SHARED ${sources} ${headers} ${mofiles} ${idl_cpp_files})
target_link_libraries(${target} CnoidUtil CnoidCorba CnoidBase)
apply_common_setting_for_plugin(${target} "${headers}")

# test program
# add_cnoid_executable(corba-test test.cpp ${idl_cpp_files})
# target_link_libraries(corba-test CnoidCorba ${OMNIDYNAMIC_LIBRARIES})

