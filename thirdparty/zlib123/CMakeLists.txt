add_library(zlib STATIC
  adler32.c
  compress.c
  crc32.c
  gzio.c
  uncompr.c
  deflate.c
  trees.c
  zutil.c
  inflate.c
  infback.c
  inftrees.c
  inffast.c
  crc32.h
  deflate.h
  inffast.h
  inffixed.h
  inflate.h
  inftrees.h
  trees.h
  zconf.h
  zlib.h
  zutil.h
  )

if(MSVC)
  set_target_properties(zlib PROPERTIES
    COMPILE_OPTIONS -Ob1
    DEBUG_POSTFIX d)
else()
  set_target_properties(zlib PROPERTIES COMPILE_OPTIONS -fPIC)
endif()

if(INSTALL_SDK)
  install(FILES zlib.h zconf.h DESTINATION ${CHOREONOID_HEADER_SUBDIR})
  install(TARGETS zlib LIBRARY DESTINATION lib ARCHIVE DESTINATION lib)
endif()
