option(BUILD_OPENRTM_PLUGIN "Building OpenRTMPlugin" OFF)

if(NOT BUILD_OPENRTM_PLUGIN)
  return()
else()
  if(NOT BUILD_CORBA_PLUGIN)
    message(FATAL_ERROR "OpenRTM Plugin requires CorbaPlugin.")
  endif()
endif()

message(STATUS "src/OpenRTMPlugin Build" )

# Plugin
set(plugin CnoidOpenRTMPlugin)

set(plugin_sources
  OpenRTMPlugin.cpp
  ConnectorCreaterDialog.cpp
  ControllerRTCItem.cpp
  BodyIoRTCItem.cpp
  OpenRTMItem.cpp
  RTCItem.cpp
  ChoreonoidExecutionContext.cpp
  ChoreonoidPeriodicExecutionContext.cpp
  ProfileHandler.cpp
  RTSNameServerView.cpp
  RTSPropertiesView.cpp
  RTSItem.cpp
  RTSDiagramView.cpp
  RTSCommonUtil.cpp
  RTSConfigurationView.cpp

  pugixml/pugixml.cpp

  deprecated/BodyRTCItem.cpp
  deprecated/VirtualRobotRTC.cpp
  deprecated/VirtualRobotPortHandler.cpp
  deprecated/BridgeConf.cpp

  LoggerUtil.cpp
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
    ConnectorCreaterDialog.h
    RTSNameServerView.h
    RTSDiagramView.h
    RTSConfigurationView.h
    OPTIONS "-DBOOST_TT_HAS_OPERATOR_HPP_INCLUDED"
  )
else()
  QT5_ADD_RESOURCES(RC_SRCS OpenRTMPlugin.qrc)
  QT5_WRAP_CPP(
    plugin_sources
    ConnectorCreaterDialog.h
    RTSDiagramView.h
    RTSNameServerView.h
    RTSConfigurationView.h
    OPTIONS "-DBOOST_TT_HAS_OPERATOR_HPP_INCLUDED"
  )
endif()

if(MSVC)
add_definitions(-D__WIN32__ -D__x86__ -D__NT__ -D__OSVERSION__=4 -D_CRT_SECURE_NO_DEPRECATE -D_WIN32_WINNT=0x0500 -DRTC_CORBA_CXXMAPPING11)
endif()

make_gettext_mofiles(${plugin} mofiles)
######
if(UNIX)
  set(OPENRTM_DIR ${OPENRTM_DIR} CACHE PATH "set the top directory of OpenRTM-aist")
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
      set(OPENRTM_INCLUDE_DIRS
        ${OPENRTM_DIR}/include 
        ${OPENRTM_DIR}/include/coil-1.1
        ${OPENRTM_DIR}/include/openrtm-1.1 
        ${OPENRTM_DIR}/include/openrtm-1.1/rtm/idl
        ${OPENRTM_DIR}/include/openrtm-1.1/rtm/ext
        ${OPENRTM_DIR}/rtm # for the Windows installer version
        ${OPENRTM_DIR}/rtm/idl # for the Windows installer version
        ${OPENRTM_DIR}
        )
      set(OPENRTM_INCLUDE_DIRS ${OPENRTM_INCLUDE_DIRS} CACHE INTERNAL "OpenRTM's include directories")
    endif()
  endif()
endif()
######
include_directories(${OPENRTM_INCLUDE_DIRS})
link_directories(${OPENRTM_LIBRARY_DIRS})
add_definitions(${OPENRTM_DEFINITIONS})

add_cnoid_plugin(${plugin} SHARED ${plugin_sources} ${plugin_headers} ${mofiles} ${RC_SRCS})

target_link_libraries(${plugin} CnoidBodyPlugin CnoidCorbaPlugin CnoidBodyIoRTC ${OPENRTM_LIBRARIES})
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
