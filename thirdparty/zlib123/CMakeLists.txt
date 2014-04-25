set(ZLIB_LIBRARY zlib)

set(sources
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
  )

set(headers
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

add_library(${ZLIB_LIBRARY} STATIC ${sources})

if(WIN32)
  set_target_properties(${ZLIB_LIBRARY} PROPERTIES COMPILE_FLAGS -Ob1)
  set_target_properties(${ZLIB_LIBRARY} PROPERTIES DEBUG_POSTFIX d )
endif() 
