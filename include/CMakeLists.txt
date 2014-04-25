
if(INSTALL_SDK)
  file(GLOB public_headers "cnoid/*")
  foreach(file ${public_headers})
    if(NOT IS_DIRECTORY ${file})
      install(FILES ${file} DESTINATION ${CNOID_HEADER_SUBDIR}/cnoid)
    endif()
  endforeach()
endif()
