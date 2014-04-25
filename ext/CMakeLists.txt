# @author Shin'ichiro Nakaoka

# if(ENABLE_INSTALL_RPATH)
#   unset(CMAKE_INSTALL_RPATH)
# endif()

file(GLOB subdirs "*")

foreach(subdir ${subdirs})
  if(EXISTS ${subdir}/CMakeLists.txt)
    add_subdirectory(${subdir})
  endif()
endforeach()
