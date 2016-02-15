# @author 

cmake_minimum_required(VERSION 2.8.3)
project(choreonoid_sdfloader_plugin)
set(target CnoidSDFLoaderPlugin)

option(BUILD_SDFLOADER_PLUGIN "Building SDFLoaderPlugin" OFF)
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -finline-functions")
option(ENABLE_GCC_FVISIBILITY_HIDDEN "Use the -fvisibility=hidden option when the sources are compiled" OFF)
option(ENABLE_INSTALL_RPATH "Enable RPATH setting for installed binary files" OFF)

string(REGEX MATCH "catkin" build_catkin "$ENV{_}")

if(build_catkin OR "${CATKIN_BUILD_BINARY_PACKAGE}")
	set(BUILD_SDFLOADER_PLUGIN ON CACHE BOOL "Building SDFLoaderPlugin" FORCE)
  set(ENABLE_GCC_FVISIBILITY_HIDDEN ON CACHE BOOL "Use the -fvisibility=hidden option when the sources are compiled" FORCE)
  set(ENABLE_INSTALL_RPATH ON CACHE BOOL "Enable RPATH setting for installed binary files" FORCE)
  set(ENV{PKG_CONFIG_PATH} $ENV{PKG_CONFIG_PATH}:${CATKIN_DEVEL_PREFIX}/lib/pkgconfig)

  find_package(catkin REQUIRED COMPONENTS
    choreonoid_ros
  )

  catkin_package(
    LIBRARIES 
    ${target}
    CATKIN_DEPENDS
    sdformat
  )
endif()

if(NOT BUILD_SDFLOADER_PLUGIN)
  return()
endif()

set(sources
  SDFLoaderPlugin.cpp
  SDFBodyLoader.cpp
  SDFLoaderPseudoGazeboColor.cpp
  SDFSensorConverter.cpp
  )

set(headers
  SDFBodyLoader.h
  SDFLoaderPseudoGazeboColor.h
  SDFSensorConverter.h
  )

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(CNOID REQUIRED choreonoid)
else()
  message(FATAL_ERROR "pkg-config is required; please install it")
endif()

execute_process(COMMAND ${PKG_CONFIG_EXECUTABLE} --variable=plugindir choreonoid
                    OUTPUT_VARIABLE CNOID_PLUGIN_SUBDIR
                    OUTPUT_STRIP_TRAILING_WHITESPACE)

set(SDFORMAT_DIR ${SDFORMAT_DIR} CACHE PATH "set the top directory of the SDFORMAT ")
if(UNIX)
  if(NOT SDFORMAT_DIR)
    pkg_check_modules(SDFORMAT REQUIRED sdformat)
  endif()
elseif(MSVC)
  if(NOT SDFORMAT_DIR)
    message(FATAL_ERROR "Please specify the directory of the SDFORMAT to SDFORMAT_DIR.")
  endif()
endif()

if(NOT SDFORMAT_FOUND)
  set(SDFORMAT_LIBRARIES sdformat)
  set(SDFORMAT_INCLUDE_DIRS ${SDFORMAT_DIR}/include)
  set(SDFORMAT_LIBRARY_DIRS ${SDFORMAT_DIR}/lib)
endif()

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CNOID_INCLUDE_DIRS}
  ${SDFORMAT_INCLUDE_DIRS}
)

link_directories(
  ${CNOID_LIBRARY_DIRS}
  ${SDFORMAT_LIBRARY_DIRS}
)

function(add_cnoid_plugin)

  add_library(${ARGV})

  if(ENABLE_GCC_FVISIBILITY_HIDDEN)
    get_target_property(compile_flags ${ARGV0} COMPILE_FLAGS)
    if(compile_flags)
      set_target_properties(${ARGV0} PROPERTIES COMPILE_FLAGS "${compile_flags} -fvisibility=hidden")
    else()
      set_target_properties(${ARGV0} PROPERTIES COMPILE_FLAGS "-fvisibility=hidden")
    endif()
  endif()

#  set_target_properties(${ARGV0} PROPERTIES
#    LIBRARY_OUTPUT_DIRECTORY ${CNOID_PLUGIN_SUBDIR}
#    ARCHIVE_OUTPUT_DIRECTORY ${CNOID_PLUGIN_SUBDIR}
#    RUNTIME_OUTPUT_DIRECTORY ${CNOID_PLUGIN_SUBDIR})

  if(ENABLE_INSTALL_RPATH)
    if(APPLE)
      set_target_properties(${ARGV0} PROPERTIES INSTALL_NAME_DIR "@rpath")
      set_target_properties(${ARGV0} PROPERTIES LINK_FLAGS "-Wl,-rpath,@loader_path,-rpath,@loader_path/..")
    else()
      set_target_properties(${ARGV0} PROPERTIES INSTALL_RPATH "$ORIGIN:$ORIGIN/..")
    endif()
  else()
    if(APPLE)
      set_target_properties(${ARGV0} PROPERTIES INSTALL_NAME_DIR "")
    else()
      set_target_properties(${ARGV0} PROPERTIES INSTALL_RPATH "$ORIGIN")
    endif()
  endif()

endfunction()

add_cnoid_plugin(${target} SHARED ${sources} ${headers})

target_link_libraries(${target} CnoidBase CnoidBody ${SDFORMAT_LIBRARIES})

install(TARGETS 
  ${target}
  RUNTIME DESTINATION ${CNOID_PLUGIN_SUBDIR}
  LIBRARY DESTINATION ${CNOID_PLUGIN_SUBDIR}
  ARCHIVE DESTINATION ${CNOID_PLUGIN_SUBDIR}
)
