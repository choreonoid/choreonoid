
# @author Shin'ichiro Nakaoka

#set(CMAKE_BUILD_TYPE Debug)
#set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -g")


option(BUILD_OPENRTM_PLUGIN "Building OpenRTMPlugin" OFF)

if(NOT BUILD_OPENRTM_PLUGIN)
  return()
else()
  if(NOT BUILD_CORBA_PLUGIN)
    message(FATAL_ERROR "OpenRTM Plugin requires CorbaPlugin.")
  endif()
endif()

set(OPENRTM_DIR ${OPENRTM_DIR} CACHE PATH "set the top directory of OpenRTM-aist")
if(UNIX)
  if(NOT OPENRTM_DIR)
    pkg_check_modules(OPENRTM REQUIRED openrtm-aist)
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
    set(OPENRTM_DIR ${OPENRTM_DIR_BACK})
    set(OPENRTM_LIBRARY_DIRS ${OPENRTM_DIR}/bin)
  else()
    set(OPENRTM_LIBRARY_DIRS ${OPENRTM_DIR}/lib)
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
      add_definitions(-DOPENRTM_VERSION110)
    else()
      file(GLOB librtc RELATIVE ${OPENRTM_LIBRARY_DIRS} "${OPENRTM_LIBRARY_DIRS}/RTC???_????.lib")
      get_filename_component(librtc ${librtc} NAME_WE)
      file(GLOB libcoil RELATIVE ${OPENRTM_LIBRARY_DIRS} "${OPENRTM_LIBRARY_DIRS}/coil???_????.lib")
      get_filename_component(libcoil ${libcoil} NAME_WE)
    endif()
    set(OPENRTM_LIBRARIES 
      optimized ${librtc} debug ${librtc}d
      optimized ${libcoil} debug ${libcoil}d
      optimized ws2_32 debug ws2_32)
    message(STATUS "bin dir = ${OPENRTM_DIR}/bin, lib dir = ${OPENRTM_LIBRARY_DIRS}, rtm libs = ${librtc}, ${libcoil}")
    install_external_libraries(${OPENRTM_DIR}/bin ${OPENRTM_LIBRARY_DIRS} ${librtc} ${libcoil})
  endif()
endif()

# Make the following variables accessible from other directories
set(OPENRTM_INCLUDE_DIRS ${OPENRTM_INCLUDE_DIRS} CACHE INTERNAL "OpenRTM's include directories")
set(OPENRTM_LIBRARY_DIRS ${OPENRTM_LIBRARY_DIRS} CACHE INTERNAL "OpenRTM's library directories")
set(OPENRTM_LIBRARIES ${OPENRTM_LIBRARIES} CACHE INTERNAL "OpenRTM's library files")

#add_definitions(${OPENRTM_CFLAGS})
include_directories(${OPENRTM_INCLUDE_DIRS})
link_directories(${OPENRTM_LIBRARY_DIRS})

set(library CnoidOpenRTM)
set(IDL_INCLUDE_DIRS 
  ${OPENRTM_PREFIX}/include/openrtm-1.1/rtm/idl
  ${OPENRTM_PREFIX}/rtm/idl # for the Windows installer version
  )

idl_compile_cpp(idl_cpp_files idl_h_files corba CameraImage PointCloud)
idl_compile_cpp(OpenHRP_idl_cpp_files OpenHRP_idl_h_files corba/OpenHRP ClockGenerator)


set(library_sources
  )

include_directories(${IDL_INCLUDE_DIRS})

add_cnoid_library(${library} STATIC 
  ${library_sources} 
  ${idl_cpp_files} ${idl_h_files}
  ${OpenHRP_idl_cpp_files} ${OpenHRP_idl_h_files})

target_link_libraries(${library} CnoidCorba)
apply_common_setting_for_library(${library} "")


# Plugin
set(plugin CnoidOpenRTMPlugin)

set(plugin_sources
  OpenRTMPlugin.cpp
  BodyRTCItem.cpp
  VirtualRobotRTC.cpp
  VirtualRobotPortHandler.cpp
  BridgeConf.cpp
  RTCItem.cpp
  OpenHRPClockGeneratorItem.cpp
  ChoreonoidExecutionContext.cpp
  )

set(plugin_headers
  )

if(MSVC)
add_definitions(-D__WIN32__ -D__x86__ -D__NT__ -D__OSVERSION__=4 -D_CRT_SECURE_NO_DEPRECATE -D_WIN32_WINNT=0x0500 -DRTC_CORBA_CXXMAPPING11)
endif()

make_gettext_mofiles(${plugin} mofiles)

add_cnoid_plugin(${plugin} SHARED ${plugin_sources} ${plugin_headers} ${mofiles})
target_link_libraries(${plugin} CnoidBodyPlugin CnoidCorbaPlugin CnoidOpenRTM ${OPENRTM_LIBRARIES})
if(UNIX)
  target_link_libraries(${plugin} ${Boost_REGEX_LIBRARY})
  add_definitions(-DOPENRTM_VERSION110)
endif()
apply_common_setting_for_plugin(${plugin} "${plugin_headers}")


if(ENABLE_PYTHON)
  add_subdirectory(python)
endif()

file(MAKE_DIRECTORY ${PROJECT_BINARY_DIR}/${CNOID_PLUGIN_SUBDIR}/rtc)
install(DIRECTORY ${PROJECT_BINARY_DIR}/${CNOID_PLUGIN_SUBDIR}/rtc DESTINATION ${CNOID_PLUGIN_SUBDIR}/rtc
  FILES_MATCHING PATTERN "*" EXCLUDE)



