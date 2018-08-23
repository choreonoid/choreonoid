if(NOT BUILD_OPENRTM_PLUGIN)
  return()
endif()

set(plugin CnoidOpenRTMPlugin)

set(plugin_sources
  OpenRTMPlugin.cpp
  PortConnectionDialog.cpp
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
  RTSNameServerView.h
  RTSDiagramView.h
  RTSConfigurationView.h
  )
set(plugin_sources ${plugin_sources} ${qtsources})

unset(idl_cpp_files)
unset(idl_h_files)
if(USE_BUILTIN_CAMERA_IMAGE_IDL)
  set(idl_files ${idl_files} CameraImage)
  idl_compile_cpp(idl_cpp_files idl_h_files deprecated/corba ${idl_files} LOCAL)
else()
  set(OPENRTM_LIBRARIES ${OPENRTM_LIBRARIES} ${OPENRTM_CAMERA_LIBRARY})
endif()
include_directories(${CMAKE_CURRENT_BINARY_DIR})

make_gettext_mofiles(${plugin} mofiles)
include_directories(${OPENRTM_INCLUDE_DIRS})
link_directories(${OPENRTM_LIBRARY_DIRS})
add_definitions(${OPENRTM_DEFINITIONS})

if(MSVC)
  add_compile_options(${OPENRTM_CFLAGS})
endif()

add_cnoid_plugin(${plugin} SHARED
  ${plugin_sources} ${plugin_headers} ${idl_cpp_files} ${idl_h_files} ${mofiles} ${RC_SRCS})
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
