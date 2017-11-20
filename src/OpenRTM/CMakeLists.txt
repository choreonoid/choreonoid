if(NOT BUILD_OPENRTM_PLUGIN)
  return()
endif()

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
    if(NOT (EXISTS "${OPENRTM_DIR}/include/openrtm-1.1/rtm/RTObjectStateMachine.h"))
      set(OPENRTM_DEFINITIONS -DOPENRTM_VERSION110)
    else()
      set(OPENRTM_DEFINITIONS -DOPENRTM_VERSION_TRUNK)
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
    if(NOT OPENRTM_VERSION STREQUAL "1.1.2")
      set(OPENRTM_DIR ${OPENRTM_DIR_BACK})
      set(OPENRTM_LIBRARY_DIRS ${OPENRTM_DIR}/bin)
    endif()
    if(NOT (EXISTS "${OPENRTM_DIR}/rtm/RTObjectStateMachine.h"))
      set(OPENRTM_DEFINITIONS -DOPENRTM_VERSION110)
    else()
      set(OPENRTM_DEFINITIONS -DOPENRTM_VERSION_TRUNK)
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
    if(NOT (EXISTS "${OPENRTM_DIR}/include/openrtm-1.1/rtm/RTObjectStateMachine.h"))
      set(OPENRTM_DEFINITIONS -DOPENRTM_VERSION110)
    endif()
  endif()
  message(STATUS "OPENRTM VERSION ${OPENRTM_VERSION}" )

  set(OPENRTM_PREFIX ${OPENRTM_DIR})

  set(OPENRTM_INCLUDE_DIRS
    ${OPENRTM_DIR}/include 
    ${OPENRTM_DIR}/include/coil-1.1
    ${OPENRTM_DIR}/include/openrtm-1.1 
    ${OPENRTM_DIR}/include/openrtm-1.1/rtm/idl
    ${OPENRTM_DIR}/rtm # for the Windows installer version
    ${OPENRTM_DIR}/rtm/idl # for the Windows installer version
    ${OPENRTM_DIR}
    )

  if(UNIX)
    set(OPENRTM_LIBRARIES RTC coil)
  elseif(MSVC)
    if(OPENRTM_VERSION STREQUAL "1.1.0")
      file(GLOB librtc RELATIVE ${OPENRTM_LIBRARY_DIRS} "${OPENRTM_LIBRARY_DIRS}/RTC???.lib")
      get_filename_component(librtc ${librtc} NAME_WE)
      file(GLOB libcoil RELATIVE ${OPENRTM_LIBRARY_DIRS} "${OPENRTM_LIBRARY_DIRS}/coil???.lib")
      get_filename_component(libcoil ${libcoil} NAME_WE)
      set(OPENRTM_DEFINITIONS -DOPENRTM_VERSION110)
      set(OPENRTM_LIBRARIES 
      optimized ${librtc} debug ${librtc}d
      optimized ${libcoil} debug ${libcoil}d
      optimized ws2_32 debug ws2_32)
    endif()
    if(OPENRTM_VERSION STREQUAL "1.1.1")
      if(CMAKE_CL_64)
        file(GLOB librtc RELATIVE ${OPENRTM_LIBRARY_DIRS} "${OPENRTM_LIBRARY_DIRS}/RTC???_????_x64.lib")
      elseif()
        file(GLOB librtc RELATIVE ${OPENRTM_LIBRARY_DIRS} "${OPENRTM_LIBRARY_DIRS}/RTC???_????.lib")
      endif()
      get_filename_component(librtc ${librtc} NAME_WE)
      if(CMAKE_CL_64)
        file(GLOB libcoil RELATIVE ${OPENRTM_LIBRARY_DIRS} "${OPENRTM_LIBRARY_DIRS}/coil???_????_x64.lib")
      elseif()
        file(GLOB libcoil RELATIVE ${OPENRTM_LIBRARY_DIRS} "${OPENRTM_LIBRARY_DIRS}/coil???_????.lib")
      endif()
      get_filename_component(libcoil ${libcoil} NAME_WE)
      set(OPENRTM_LIBRARIES 
      optimized ${librtc} debug ${librtc}d
      optimized ${libcoil} debug ${libcoil}d
      optimized ws2_32 debug ws2_32)
    endif()
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
set(target1 CnoidOpenRTM)
set(IDL_INCLUDE_DIRS 
  ${OPENRTM_PREFIX}/include/openrtm-1.1/rtm/idl
  ${OPENRTM_PREFIX}/rtm/idl # for the Windows installer version
  )
idl_compile_cpp(idl_cpp_files idl_h_files corba CameraCommonInterface PointCloud)
add_cnoid_library(${target1} STATIC ${idl_cpp_files} ${idl_h_files})
apply_common_setting_for_library(${target1} "")


# BodyIoRTC
set(target2 CnoidBodyIoRTC)
set(sources BodyIoRTC.cpp)
set(headers BodyIoRTC.h exportdecl.h)

if(MSVC)
  add_definitions(-DRTC_CORBA_CXXMAPPING11)
endif()

add_cnoid_library(${target2} SHARED ${sources} ${headers})
target_link_libraries(${target2} CnoidBody CnoidOpenRTM CnoidCorba ${OPENRTM_LIBRARIES})
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
