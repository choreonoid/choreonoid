
# @author Shin'ichiro Nakaoka

#set(CMAKE_BUILD_TYPE Debug)

option(BUILD_BALANCER_PLUGIN "Building BalancerPlugin" ON)
if(NOT BUILD_BALANCER_PLUGIN)
  return()
elseif(NOT BUILD_POSE_SEQ_PLUGIN)
  message(FATAL_ERROR "BalancerPlugin requires PoseSeqPlugin.")
endif()

set(target CnoidBalancerPlugin)

set(sources
  BalancerPlugin.cpp
  WaistBalancer.cpp
)

set(headers
  )

make_gettext_mofiles(${target} mofiles)
add_cnoid_plugin(${target} SHARED ${sources} ${headers} ${mofiles})

add_dependencies(${target} CnoidPoseSeqPlugin)

if(WIN32)
  set_target_properties(${target} PROPERTIES COMPILE_DEFINITIONS "CNOID_USE_GETTEXT_WRAPPER")
  target_link_libraries(${target} CnoidBase CnoidPoseSeqPlugin )
elseif(APPLE)
  if(ENABLE_INSTALL_RPATH)
    set_target_properties(${target} PROPERTIES COMPILE_DEFINITIONS "CNOID_USE_GETTEXT_WRAPPER")
    target_link_libraries(${target}
      ${QT_QTCORE_LIBRARY}
      ${QT_QTGUI_LIBRARY}
      libCnoidUtil.dylib
      libCnoidBase.dylib
      libCnoidBody.dylib
      libCnoidBodyPlugin.dylib
      libCnoidPoseSeqPlugin.dylib
      )
  else()
    target_link_libraries(${target} CnoidBase CnoidPoseSeqPlugin)
  endif()
else()
  target_link_libraries(${target} CnoidBase CnoidPoseSeqPlugin)
endif()

apply_common_setting_for_plugin(${target} "${headers}")
