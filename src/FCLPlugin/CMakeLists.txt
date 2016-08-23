
option(BUILD_FCL_PLUGIN "Building FCLPlugin" OFF)

if(NOT BUILD_FCL_PLUGIN)
  return()
endif()

#set(CMAKE_BUILD_TYPE Debug)

set(FCL_DIR ${FCL_DIR} CACHE PATH "set the top directory of the FCL ")
if(UNIX)
  if(NOT FCL_DIR)
    pkg_check_modules(FCL REQUIRED fcl)
  endif()
elseif(MSVC)
  if(NOT FCL_DIR)
    message(FATAL_ERROR "Please specify the directory of the FCL to FCL_DIR.")
  endif()
endif()
set(CCD_DIR ${CCD_DIR} CACHE PATH "set the top directory of the CCD ")
if(UNIX)
  if(NOT CCD_DIR)
    pkg_check_modules(CCD REQUIRED ccd)
  endif()
elseif(MSVC)
  if(NOT CCD_DIR)
    message(FATAL_ERROR "Please specify the directory of the CCD to CCD_DIR.")
  endif()
endif()


if(FCL_DIR)
  if(UNIX)
  set(FCL_LIBRARIES fcl ccd)
  elseif(MSVC)
    set(FCL_LIBRARIES optimized fcl debug fcld
                      optimized ccd debug ccdd)
  endif()
  set(FCL_INCLUDE_DIRS ${FCL_DIR}/include )
  set(FCL_LIBRARY_DIRS ${FCL_DIR}/lib/)
  set(CCD_LIBRARY_DIRS ${CCD_DIR}/lib/)
endif()

include_directories(${FCL_INCLUDE_DIRS})
link_directories(${FCL_LIBRARY_DIRS} ${CCD_LIBRARY_DIRS})

set(target CnoidFCLPlugin)

set(sources
  FCLCollisionDetector.cpp
  )

set(headers
  FCLCollisionDetector.h
  )

#make_gettext_mofiles(${target} mofiles)
add_cnoid_plugin(${target} SHARED ${sources} ${headers} ${mofiles})
target_link_libraries(${target} CnoidBodyPlugin ${FCL_LIBRARIES})
set_target_properties(${target} PROPERTIES COMPILE_FLAGS "-std=c++11")
apply_common_setting_for_plugin(${target} "${headers}")
install_external_libraries(${FCL_DIR}/bin ${FCL_DIR}/lib ${FCL_LIBRARIES})
