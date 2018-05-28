option(BUILD_OPENRTM_PLUGIN "Building OpenRTMPlugin" OFF)

if(NOT BUILD_OPENRTM_PLUGIN)
  return()
else()
  if(NOT BUILD_CORBA_PLUGIN)
    message(FATAL_ERROR "OpenRTM Plugin requires CorbaPlugin.")
  endif()
endif()

# Plugin
set(plugin CnoidOpenRTMPlugin)

set(plugin_sources
  OpenRTMPlugin.cpp
  ConnectorCreaterDialog.cpp
  ControllerRTCItem.cpp
  BodyIoRTCItem.cpp
  OpenRTMItem.cpp
  RTCItem.cpp
  SimulationExecutionContext.cpp
  SimulationPeriodicExecutionContext.cpp
  ProfileHandler.cpp
  RTSNameServerView.cpp
  RTSPropertiesView.cpp
  RTSItem.cpp
  RTSDiagramView.cpp
  RTMImageView.cpp
  RTSCommonUtil.cpp
  RTSConfigurationView.cpp
  VisionSensorSubscriberRTCItem.cpp
  LoggerUtil.cpp
  deprecated/BodyRTCItem.cpp
  deprecated/VirtualRobotRTC.cpp
  deprecated/VirtualRobotPortHandler.cpp
  deprecated/ChoreonoidExecutionContext.cpp
  deprecated/ChoreonoidPeriodicExecutionContext.cpp
  deprecated/BridgeConf.cpp
  deprecated/PointCloudSubscriberRTCItem.cpp
  )

set(plugin_headers
  )

QTX_ADD_RESOURCES(RC_SRCS OpenRTMPlugin.qrc)
QTX_WRAP_CPP(
  qtsources
  ConnectorCreaterDialog.h
  RTSNameServerView.h
  RTSDiagramView.h
  RTSConfigurationView.h
  )
set(plugin_sources ${plugin_sources} ${qtsources})

if(MSVC)
add_definitions(-D__WIN32__ -D__x86__ -D__NT__ -D__OSVERSION__=4 -D_CRT_SECURE_NO_DEPRECATE -D_WIN32_WINNT=0x0500 -DRTC_CORBA_CXXMAPPING11)
endif()

set(idl_files PointCloud)

option(USE_BUILTIN_CAMERA_IMAGE_IDL "Enable this option when you run hrpsys-base on Choreonoid" OFF)
if(USE_BUILTIN_CAMERA_IMAGE_IDL)
  list(APPEND OPENRTM_DEFINITIONS -DUSE_BUILTIN_CAMERA_IMAGE_IDL)
  set(idl_files ${idl_files} CameraImage)
else()
  set(OPENRTM_LIBRARIES ${OPENRTM_LIBRARIES} rtmCamera)
endif()

idl_compile_cpp(idl_cpp_files idl_h_files deprecated/corba ${idl_files} LOCAL)

make_gettext_mofiles(${plugin} mofiles)

include_directories(${OPENRTM_INCLUDE_DIRS})
link_directories(${OPENRTM_LIBRARY_DIRS})
add_definitions(${OPENRTM_DEFINITIONS})

add_cnoid_plugin(${plugin} SHARED ${plugin_sources} ${plugin_headers} ${idl_cpp_files} ${idl_h_files} ${mofiles} ${RC_SRCS})

target_link_libraries(${plugin} CnoidBodyPlugin CnoidCorbaPlugin CnoidBodyIoRTC ${OPENRTM_LIBRARIES} pugixml)
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
