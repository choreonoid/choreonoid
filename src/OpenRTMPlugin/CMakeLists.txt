option(BUILD_OPENRTM_PLUGIN "Building OpenRTMPlugin" OFF)

if(NOT BUILD_OPENRTM_PLUGIN)
  return()
else()
  if(NOT BUILD_CORBA_PLUGIN)
    message(FATAL_ERROR "OpenRTM Plugin requires CorbaPlugin.")
  endif()
endif()

#function(add_cnoid_openRTM_plugin)
#
#  add_library(${ARGV})
#
#  set_target_properties(${ARGV0} PROPERTIES
#     LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/${CNOID_PLUGIN_SUBDIR}
#     ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/${CNOID_PLUGIN_SUBDIR}
#     RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/${CNOID_PLUGIN_SUBDIR})

#   if(ENABLE_INSTALL_RPATH)
#     if(APPLE)
#       set_target_properties(${ARGV0} PROPERTIES INSTALL_NAME_DIR "@rpath")
#       set_target_properties(${ARGV0} PROPERTIES LINK_FLAGS "-Wl,-rpath,@loader_path,-rpath,@loader_path/..")
#     else()
#       set_target_properties(${ARGV0} PROPERTIES INSTALL_RPATH "$ORIGIN:$ORIGIN/..")
#     endif()
#   else()
#     if(APPLE)
#       set_target_properties(${ARGV0} PROPERTIES INSTALL_NAME_DIR "")
#     else()
#       set_target_properties(${ARGV0} PROPERTIES INSTALL_RPATH "$ORIGIN")
#     endif()
#   endif()

# endfunction()


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

include_directories(${OPENRTM_INCLUDE_DIRS})
link_directories(${OPENRTM_LIBRARY_DIRS})
add_definitions(${OPENRTM_DEFINITIONS})

add_cnoid_plugin(${plugin} SHARED ${plugin_sources} ${plugin_headers} ${mofiles} ${RC_SRCS})

# if(OPENRTM_VERSION STREQUAL "1.1.0")
#   add_cnoid_plugin(${plugin} SHARED ${plugin_sources} ${plugin_headers} ${mofiles}  ${RC_SRCS})
# else()
#   add_cnoid_OpenRTM_plugin(${plugin} SHARED ${plugin_sources} ${plugin_headers} ${mofiles} ${RC_SRCS})
#   if(ENABLE_GCC_FVISIBILITY_HIDDEN)
#     set_source_files_properties(BodyRTCItem.cpp
#                                 VirtualRobotRTC.cpp
#                                 VirtualRobotPortHandler.cpp
#                                 BridgeConf.cpp
#                                 RTCItem.cpp
# 				ControllerRTCItem.cpp
# 				BodyIoRTCItem.cpp
#                                 ChoreonoidExecutionContext.cpp 
#                                 PROPERTIES COMPILE_FLAGS "-fvisibility=hidden")
#   endif()
# endif()

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
