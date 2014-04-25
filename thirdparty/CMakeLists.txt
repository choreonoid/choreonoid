
add_subdirectory(yaml-0.1.3)
add_subdirectory(glew-1.9.0)
add_subdirectory(irrxml-1.2)

if(MSVC)
  add_subdirectory(lpng1232)
  add_subdirectory(Jpeg-6b)
  add_subdirectory(zlib123)
endif()

# Copy VC++ runtimes
foreach(file msvcp100.dll msvcr100.dll)
  set(path ${CMAKE_CURRENT_SOURCE_DIR}/windows/bin/${file})
  if(EXISTS ${path})
    install(PROGRAMS ${path} DESTINATION bin)
  endif()
endforeach()
