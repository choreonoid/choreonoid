
# Copy VC++ runtimes
foreach(file msvcp100.dll msvcr100.dll  libbz2.dll)
  set(path ${CMAKE_CURRENT_SOURCE_DIR}/bin/${file})
  if(EXISTS ${path})
    install(PROGRAMS ${path} DESTINATION bin)
  endif()
endforeach()
