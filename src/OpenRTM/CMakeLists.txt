if(NOT BUILD_OPENRTM_PLUGIN)
  return()
endif()

message(STATUS "src/OpenRTM Build" )

set(OPENRTM_DIR ${OPENRTM_DIR} CACHE PATH "set the top directory of OpenRTM-aist")
if(UNIX)
  if(NOT OPENRTM_DIR)
    pkg_check_modules(OPENRTM REQUIRED openrtm-aist)
    message(STATUS "OPENRTM VERSION ${OPENRTM_VERSION}" )
    execute_process(
      COMMAND pkg-config --variable=prefix openrtm-aist
      OUTPUT_VARIABLE OPENRTM_DIR
      RESULT_VARIABLE RESULT
      OUTPUT_STRIP_TRAILING_WHITESPACE)
    message(STATUS "OPENRTM_DIR=${OPENRTM_DIR}")
    if(OPENRTM_VERSION STRLESS "1.2.0")
      set(OPENRTM_DEFINITIONS -DOPENRTM_VERSION11)
    endif()
  endif()
elseif(MSVC)
  if(NOT OPENRTM_DIR)
    message(FATAL_ERROR "Please specify the directory of OpenRTM-aist to OPENRTM_DIR.")
  endif()
endif()

if(OPENRTM_DIR)
  if(MSVC)
    set(OPENRTM_DIR_BACK ${OPENRTM_DIR})
    include(${OPENRTM_DIR}/cmake/OpenRTMConfig.cmake)
    set(OPENRTM_EXT_LIBRARIES 
      optimized rtmCamera${OPENRTM_SHORT_VERSION}_$ENV{RTM_VC_VERSION}_x64 debug rtmCamera${OPENRTM_SHORT_VERSION}_$ENV{RTM_VC_VERSION}_x64d
    )
    set(OPENRTM_LIBRARIES ${OPENRTM_LIBRARIES} ${OPENRTM_EXT_LIBRARIES})
    if(OPENRTM_VERSION STRLESS "1.2.0")
      set(OPENRTM_DEFINITIONS -DOPENRTM_VERSION11)
    endif()
  else()
    if(EXISTS "${OPENRTM_DIR}/lib/x86_64-linux-gnu/openrtm-1.1/cmake/OpenRTMConfig.cmake")
      include(${OPENRTM_DIR}/lib/x86_64-linux-gnu/openrtm-1.1/cmake/OpenRTMConfig.cmake)
    elseif(EXISTS "${OPENRTM_DIR}/lib/i386-linux-gnu/openrtm-1.1/cmake/OpenRTMConfig.cmake")
      include(${OPENRTM_DIR}/lib/i386-linux-gnu/openrtm-1.1/cmake/OpenRTMConfig.cmake)
    else()
      include(${OPENRTM_DIR}/lib/openrtm-1.1/cmake/OpenRTMConfig.cmake)
    endif()
    set(OPENRTM_LIBRARY_DIRS ${OPENRTM_DIR}/lib)
    if(OPENRTM_VERSION STRLESS "1.2.0")
      set(OPENRTM_DEFINITIONS -DOPENRTM_VERSION11)
    endif()
  endif()
  message(STATUS "OPENRTM VERSION ${OPENRTM_VERSION}" )

  set(OPENRTM_PREFIX ${OPENRTM_DIR})

  set(OPENRTM_INCLUDE_DIRS
    ${OPENRTM_DIR}/include 
    ${OPENRTM_DIR}/include/coil-${OPENRTM_VERSION_MAJOR}.${OPENRTM_VERSION_MINOR}
    ${OPENRTM_DIR}/include/openrtm-${OPENRTM_VERSION_MAJOR}.${OPENRTM_VERSION_MINOR}
    ${OPENRTM_DIR}/include/openrtm-${OPENRTM_VERSION_MAJOR}.${OPENRTM_VERSION_MINOR}/rtm/idl
    ${OPENRTM_DIR}/include/openrtm-${OPENRTM_VERSION_MAJOR}.${OPENRTM_VERSION_MINOR}/rtm/ext
    ${OPENRTM_DIR}/rtm # for the Windows installer version
    ${OPENRTM_DIR}/rtm/idl # for the Windows installer version
    ${OPENRTM_DIR}
    )

  if(UNIX)
    set(OPENRTM_LIBRARIES RTC coil)
  elseif(MSVC)
    message(STATUS "bin dir = ${OPENRTM_DIR}/bin, lib dir = ${OPENRTM_LIBRARY_DIRS}, rtm libs = ${OPENRTM_LIBRARIES}")
    install_external_libraries(${OPENRTM_LIBRARY_DIRS} ${OPENRTM_LIBRARY_DIRS} ${OPENRTM_LIBRARIES})
  endif()
endif()

# Make the following variables accessible from other directories
set(OPENRTM_INCLUDE_DIRS ${OPENRTM_INCLUDE_DIRS} CACHE INTERNAL "OpenRTM's include directories")
set(OPENRTM_LIBRARY_DIRS ${OPENRTM_LIBRARY_DIRS} CACHE INTERNAL "OpenRTM's library directories")
set(OPENRTM_LIBRARIES ${OPENRTM_LIBRARIES} CACHE INTERNAL "OpenRTM's library files")

set(OPENRTM_DEFINITIONS ${OPENRTM_DEFINITIONS} PARENT_SCOPE)
add_definitions(${OPENRTM_DEFINITIONS})
include_directories(${OPENRTM_INCLUDE_DIRS})
link_directories(${OPENRTM_LIBRARY_DIRS})


# Port data type stubs
set(IDL_INCLUDE_DIRS 
  ${OPENRTM_PREFIX}/include/openrtm-${OPENRTM_VERSION_MAJOR}.${OPENRTM_VERSION_MINOR}/rtm/idl
  ${OPENRTM_PREFIX}/rtm/idl # for the Windows installer version
  )

# BodyIoRTC
set(target2 CnoidBodyIoRTC)
set(sources BodyIoRTC.cpp)
set(headers BodyIoRTC.h exportdecl.h)

if(MSVC)
  add_definitions(-DRTC_CORBA_CXXMAPPING11)
endif()

add_cnoid_library(${target2} SHARED ${sources} ${headers})
target_link_libraries(${target2} CnoidBody CnoidCorba ${OPENRTM_LIBRARIES})
apply_common_setting_for_library(${target2} "${headers}")


function(add_cnoid_body_io_rtc)

  set(target ${ARGV0})
  list(REMOVE_AT ARGV 0)

  add_library(${target} SHARED ${ARGV})
  target_link_libraries(${target} CnoidBodyIoRTC)
  
  if(MSVC)
    add_definitions(-DRTC_CORBA_CXXMAPPING11)
  endif()

  set_target_properties(${target} PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/${CNOID_PLUGIN_SUBDIR}/rtc
    PREFIX "")
  if(ENABLE_INSTALL_RPATH)
    set_target_properties(${target} PROPERTIES INSTALL_RPATH "$ORIGIN/../..")
  endif()

  install(TARGETS ${target}
    RUNTIME DESTINATION ${CNOID_PLUGIN_SUBDIR}/rtc CONFIGURATIONS Release Debug
    LIBRARY DESTINATION ${CNOID_PLUGIN_SUBDIR}/rtc CONFIGURATIONS Release Debug)

endfunction()
