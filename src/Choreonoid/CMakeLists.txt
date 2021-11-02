option(BUILD_CHOREONOID_EXECUTABLE "Building the executable file of Choreonoid" ON)

if(BUILD_CHOREONOID_EXECUTABLE)
  set(choreonoid_src choreonoid.cpp)
  if(WIN32)
    set(choreonoid_src ${choreonoid_src} choreonoid.rc)
  endif()
  
  choreonoid_add_executable(choreonoid ${choreonoid_src})
  target_link_libraries(choreonoid CnoidBase)
  set_target_properties(choreonoid PROPERTIES PROJECT_LABEL Application)
  
  if(MSVC)
    set_target_properties(choreonoid PROPERTIES DEBUG_POSTFIX -debug)
    if(CHOREONOID_USE_SUBSYSTEM_CONSOLE)
      set_target_properties(choreonoid PROPERTIES LINK_FLAGS "/SUBSYSTEM:CONSOLE")
    else()
      set_target_properties(choreonoid PROPERTIES WIN32_EXECUTABLE true)
    endif()
  endif()

  if(UNIX)
    install(FILES choreonoid.1 DESTINATION share/man/man1)
  endif()
endif()


choreonoid_add_executable(choreonoid-info choreonoid-info.cpp)
target_link_libraries(choreonoid-info CnoidUtil)
if(MSVC)
  if(CHOREONOID_USE_SUBSYSTEM_CONSOLE)
    set_target_properties(choreonoid-info PROPERTIES LINK_FLAGS "/SUBSYSTEM:CONSOLE")
  endif()  
endif()
