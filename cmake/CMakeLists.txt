set(include_dirs ${CHOREONOID_SDK_INCLUDE_DIRS})
list(REMOVE_DUPLICATES include_dirs)

set(library_dirs ${CHOREONOID_SDK_LIBRARY_DIRS})
list(REMOVE_DUPLICATES library_dirs)

get_directory_property(compile_definitions COMPILE_DEFINITIONS)

set(modules
  ${CMAKE_CURRENT_BINARY_DIR}/ChoreonoidConfig.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/ChoreonoidConfigVersion.cmake
  ChoreonoidBasicBuildFunctions.cmake
  ChoreonoidFindQt.cmake
  )

set(util_dependencies fmt)

set(has_boost_libs_for_util_libs true)
set(boost_version ${Boost_MAJOR_VERSION}.${Boost_MINOR_VERSION}.${Boost_SUBMINOR_VERSION})
# set(boost_version ${Boost_VERSION_STRING})
set(boost_components_for_util_libs system)
set(boost_libs_for_util_libs ${Boost_SYSTEM_LIBRARY})

if(CMAKE_CXX_STANDARD LESS 17)
  list(APPEND boost_components_for_util_libs filesystem)
  list(APPEND boost_libs_for_util_libs ${Boost_FILESYSTEM_LIBRARY})
else() # C++17
  if(UNIX)
    list(APPEND util_dependencies stdc++fs)
  endif()
endif()

if(UNIX)
  set(CHOREONOID_USE_SUBSYSTEM_CONSOLE false)
elseif(MSVC)
  if(ENABLE_GETTEXT)
    list(APPEND util_dependencies ${GETTEXT_LIBRARIES})
  endif()
endif()

if(ENABLE_PYTHON)
  list(APPEND modules ChoreonoidPythonBuildFunctions.cmake)
endif()

if(ENABLE_CORBA AND BUILD_CORBA_PLUGIN)
  set(is_corba_enabled true)
  list(APPEND modules ChoreonoidCorbaBuildFunctions.cmake)
endif()

install(FILES ${modules} DESTINATION ${CHOREONOID_CMAKE_CONFIG_SUBDIR})
if(UNIX)
  string(CONCAT code
    "execute_process(COMMAND ${CMAKE_COMMAND} "
    "-E create_symlink ChoreonoidConfig.cmake choreonoid-config.cmake "
    "WORKING_DIRECTORY ${CMAKE_INSTALL_PREFIX}/${CHOREONOID_CMAKE_CONFIG_SUBDIR})")
  install(CODE ${code})
  string(CONCAT version-script
    "execute_process(COMMAND ${CMAKE_COMMAND} "
    "-E create_symlink ChoreonoidConfigVersion.cmake choreonoid-config-version.cmake "
    "WORKING_DIRECTORY ${CMAKE_INSTALL_PREFIX}/${CHOREONOID_CMAKE_CONFIG_SUBDIR})")
  install(CODE ${version-script})
endif()

configure_file(ChoreonoidConfig.cmake.in ChoreonoidConfig.cmake @ONLY)
configure_file(ChoreonoidConfigVersion.cmake.in ChoreonoidConfigVersion.cmake @ONLY)
