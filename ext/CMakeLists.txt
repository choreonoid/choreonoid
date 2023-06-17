# @author Shin'ichiro Nakaoka

file(GLOB subdirs "*")
list(SORT subdirs)

foreach(subdir ${subdirs})
  if(NOT subdir MATCHES "\\.disabled$" AND EXISTS ${subdir}/CMakeLists.txt)
    add_subdirectory(${subdir})
  endif()
endforeach()
