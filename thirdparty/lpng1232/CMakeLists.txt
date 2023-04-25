include_directories(${CMAKE_CURRENT_SOURCE_DIR})

add_library(png STATIC
  png.c
  pngerror.c
  pngget.c
  pngmem.c
  pngpread.c
  pngread.c
  pngrio.c
  pngrtran.c
  pngrutil.c
  pngset.c
  pngtrans.c
  pngwio.c
  pngwrite.c
  pngwtran.c
  pngwutil.c
  png.h
  pngconf.h)

if(MSVC)
  set_target_properties(png PROPERTIES
    COMPILE_DEFINITIONS "PNG_NO_MMX_CODE"
    DEBUG_POSTFIX d)
else()
  set_target_properties(png PROPERTIES COMPILE_OPTIONS -fPIC)
endif()

if(INSTALL_SDK)
  install(FILES png.h pngconf.h DESTINATION ${CHOREONOID_HEADER_SUBDIR})
  install(TARGETS png LIBRARY DESTINATION lib ARCHIVE DESTINATION lib)
endif()
