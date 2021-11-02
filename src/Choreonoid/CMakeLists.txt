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
    if(CMAKE_INSTALL_PREFIX MATCHES "^$ENV{HOME}/.+") # Inside home directory?
      # Execute update-mime-database ~/.local/share/mime to recognize the mime type 
      install(FILES application-x-choreonoid.xml DESTINATION $ENV{HOME}/.local/share/mime/packages)
      # Execute update-desktop-database ~/.local/share/applications to recognize the application
      configure_file(choreonoid.desktop.in choreonoid.desktop @ONLY)
      install(FILES ${CMAKE_CURRENT_BINARY_DIR}/choreonoid.desktop DESTINATION $ENV{HOME}/.local/share/applications)
      # Execute update-icon-caches ~/.local/share/icons/* to recognize the icon
      install(FILES ${PROJECT_SOURCE_DIR}/src/Base/icon/choreonoid.svg DESTINATION $ENV{HOME}/.local/share/icons/hicolor/scalable/mimetypes RENAME application-x-choreonoid.svg)
    endif()

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
