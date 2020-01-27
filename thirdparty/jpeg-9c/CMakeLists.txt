add_library(jpeg STATIC
  jaricom.c
  jcapimin.c
  jcapistd.c
  jcarith.c
  jccoefct.c
  jccolor.c
  jcdctmgr.c
  jchuff.c
  jcinit.c
  jcmainct.c
  jcmarker.c
  jcmaster.c
  jcomapi.c
  jcparam.c
  jcprepct.c
  jcsample.c
  jctrans.c
  jdapimin.c
  jdapistd.c
  jdarith.c
  jdatadst.c
  jdatasrc.c
  jdcoefct.c
  jdcolor.c
  jddctmgr.c
  jdhuff.c
  jdinput.c
  jdmainct.c
  jdmarker.c
  jdmaster.c
  jdmerge.c
  jdpostct.c
  jdsample.c
  jdtrans.c
  jerror.c
  jfdctflt.c
  jfdctfst.c
  jfdctint.c
  jidctflt.c
  jidctfst.c
  jidctint.c
  jquant1.c
  jquant2.c
  jutils.c
  jmemmgr.c
  jmemnobs.c
  jconfig.h
  jdct.h
  jerror.h
  jinclude.h
  jmemsys.h
  jmorecfg.h
  jpegint.h
  jpeglib.h
  jversion.h
  cdjpeg.h
  cderror.h
  transupp.h
  )

if(MSVC)
  set_target_properties(jpeg PROPERTIES
    COMPILE_OPTIONS -Ob1
    DEBUG_POSTFIX d)
else()
  set_target_properties(jpeg PROPERTIES COMPILE_OPTIONS -fPIC)
endif()

if(INSTALL_SDK)
  install(FILES jpeglib.h jconfig.h jmorecfg.h DESTINATION ${CHOREONOID_HEADER_SUBDIR})
  install(TARGETS jpeg LIBRARY DESTINATION lib ARCHIVE DESTINATION lib)
endif()
