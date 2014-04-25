
if(NOT USE_EXTERNAL_GLEW)

  set(target glew32)

  set(headers
    include/GL/glew.h
    include/GL/wglew.h
    )
    
  set(sources
    src/glew.c
    )

  add_cnoid_library(${target} SHARED ${sources} ${headers})

  set_target_properties(${target} PROPERTIES
    COMPILE_DEFINITIONS "HAVE_CONFIG_H;GLEW_BUILD;"
    DEBUG_POSTFIX d
    )

  if(MSVC)
    target_link_libraries(${target} opengl32)
  endif()

  apply_common_setting_for_ext_library(${target} "${headers}")

  if(INSTALL_SDK)
    install(FILES ${headers} DESTINATION ${CNOID_HEADER_SUBDIR}/GL)
  endif()

endif()
