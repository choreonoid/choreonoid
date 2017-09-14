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

function(add_cnoid_openRTM_plugin)

  add_library(${ARGV})

  set_target_properties(${ARGV0} PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/${CNOID_PLUGIN_SUBDIR}
    ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/${CNOID_PLUGIN_SUBDIR}
    RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/${CNOID_PLUGIN_SUBDIR})

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
      add_definitions(-DOPENRTM_VERSION110)
    else()
      add_definitions(-DOPENRTM_VERSION_TRUNK)
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
      add_definitions(-DOPENRTM_VERSION110)
    else()
      add_definitions(-DOPENRTM_VERSION_TRUNK)
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
      add_definitions(-DOPENRTM_VERSION110)
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
      add_definitions(-DOPENRTM_VERSION110)
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


add_subdirectory(library)


# Plugin
set(plugin CnoidOpenRTMPlugin)

set(plugin_sources
  OpenRTMPlugin.cpp
  ControllerRTCItem.cpp
  BodyIoRTCItem.cpp
  RTCItem.cpp
  ChoreonoidExecutionContext.cpp
  ChoreonoidPeriodicExecutionContext.cpp
  RTSNameServerView.cpp
  RTSPropertiesView.cpp
  RTSItem.cpp
  RTSDiagramView.cpp
  RTSCommonUtil.cpp

  deprecated/BodyRTCItem.cpp
  deprecated/VirtualRobotRTC.cpp
  deprecated/VirtualRobotPortHandler.cpp
  deprecated/BridgeConf.cpp
  deprecated/OpenHRPClockGeneratorItem.cpp
  )

set(plugin_headers
  )

if(NOT QT5)
  QT4_ADD_RESOURCES(RC_SRCS OpenRTMPlugin.qrc)
else()
  QT5_ADD_RESOURCES(RC_SRCS OpenRTMPlugin.qrc)
endif()

if(NOT QT5)
  QT4_ADD_RESOURCES(RC_SRCS OpenRTMPlugin.qrc)
  QT4_WRAP_CPP(
    plugin_sources
    RTSNameServerView.h
    RTSDiagramView.h
    OPTIONS "-DBOOST_TT_HAS_OPERATOR_HPP_INCLUDED"
  )
else()
  QT5_ADD_RESOURCES(RC_SRCS OpenRTMPlugin.qrc)
  QT5_WRAP_CPP(
    plugin_sources
    RTSDiagramView.h
    RTSNameServerView.h
    OPTIONS "-DBOOST_TT_HAS_OPERATOR_HPP_INCLUDED"
  )
endif()

if(MSVC)
add_definitions(-D__WIN32__ -D__x86__ -D__NT__ -D__OSVERSION__=4 -D_CRT_SECURE_NO_DEPRECATE -D_WIN32_WINNT=0x0500 -DRTC_CORBA_CXXMAPPING11)
endif()

make_gettext_mofiles(${plugin} mofiles)

if(OPENRTM_VERSION STREQUAL "1.1.0")
  add_cnoid_plugin(${plugin} SHARED ${plugin_sources} ${plugin_headers} ${mofiles}  ${RC_SRCS})
else()
  add_cnoid_OpenRTM_plugin(${plugin} SHARED ${plugin_sources} ${plugin_headers} ${mofiles} ${RC_SRCS})
  if(ENABLE_GCC_FVISIBILITY_HIDDEN)
    set_source_files_properties(BodyRTCItem.cpp
                                VirtualRobotRTC.cpp
                                VirtualRobotPortHandler.cpp
                                BridgeConf.cpp
                                RTCItem.cpp
				ControllerRTCItem.cpp
				BodyIoRTCItem.cpp
                                OpenHRPClockGeneratorItem.cpp
                                ChoreonoidExecutionContext.cpp 
                                PROPERTIES COMPILE_FLAGS "-fvisibility=hidden")
  endif()
endif()

target_link_libraries(${plugin} CnoidBodyPlugin CnoidCorbaPlugin CnoidOpenRTM CnoidBodyIoRTC ${OPENRTM_LIBRARIES})
if(UNIX)
  target_link_libraries(${plugin} ${Boost_REGEX_LIBRARY})
endif()
apply_common_setting_for_plugin(${plugin} "${plugin_headers}")

if(ENABLE_PYTHON)
  if(USE_PYBIND11)
    add_subdirectory(pybind11)
  elseif(USE_BOOST_PYTHON)
    add_subdirectory(boostpython)
  endif()
endif()

file(MAKE_DIRECTORY ${PROJECT_BINARY_DIR}/${CNOID_PLUGIN_SUBDIR}/rtc)
install(DIRECTORY ${PROJECT_BINARY_DIR}/${CNOID_PLUGIN_SUBDIR}/rtc DESTINATION ${CNOID_PLUGIN_SUBDIR}/rtc
  FILES_MATCHING PATTERN "*" EXCLUDE)

if(QT5)
  qt5_use_modules(${plugin} Core OpenGL Network)
endif()
