option(BUILD_FCL_PLUGIN "Building the Flexible Collision Library (FCL) Plugin" OFF)
mark_as_advanced(BUILD_FCL_PLUGIN)

if(NOT BUILD_FCL_PLUGIN)
  return()
endif()

set(FCL_DIR ${FCL_DIR} CACHE PATH "set the top directory of the FCL ")
if(NOT FCL_DIR)
  pkg_check_modules(FCL fcl)
  if(NOT FCL_FOUND)
    message(FATAL_ERROR "Please specify the directory of the FCL to FCL_DIR.")
  endif()
else()
  set(FCL_PREFIX = ${FCL_DIR})
  set(FCL_INCLUDE_DIRS ${FCL_DIR}/include)
  set(FCL_LIBRARY_DIRS ${FCL_DIR}/lib)
  if(UNIX)
    set(FCL_LIBRARIES fcl)
  elseif(MSVC)
    set(FCL_LIBRARIES optimized fcl debug fcld)
  endif()
endif()

include_directories(${FCL_INCLUDE_DIRS})
link_directories(${FCL_LIBRARY_DIRS})

set(target CnoidFCLPlugin)
choreonoid_add_plugin(${target} FCLCollisionDetector.cpp)
target_link_libraries(${target} PUBLIC CnoidBodyPlugin PRIVATE ${FCL_LIBRARIES})
install_runtime_dlls(${FCL_PREFIX}/bin ${FCL_LIBRARIES})
