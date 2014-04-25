# @author Shin'ichiro Nakaoka

function(install_sample_source)
  if(INSTALL_SDK)
    get_filename_component(name ${CMAKE_CURRENT_SOURCE_DIR} NAME)
    foreach(file ${ARGV})
      install(FILES ${file} DESTINATION ${CNOID_SHARE_SUBDIR}/sample/${name}/)
    endforeach()
    install(FILES ManualMakefile DESTINATION ${CNOID_SHARE_SUBDIR}/sample/${name} RENAME Makefile)
  endif()
endfunction()

file(GLOB subdirs "*")

foreach(subdir ${subdirs})
  if(EXISTS ${subdir}/CMakeLists.txt)
    add_subdirectory(${subdir})
  endif()
endforeach()
