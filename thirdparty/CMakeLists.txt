


# Copy VC++ runtimes
foreach(file msvcp100.dll msvcr100.dll)
  set(path ${CMAKE_CURRENT_SOURCE_DIR}/windows/bin/${file})
  if(EXISTS ${path})
    install(PROGRAMS ${path} DESTINATION bin)
  endif()
endforeach()
