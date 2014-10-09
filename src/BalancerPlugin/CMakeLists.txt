
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

if(QT5)
  qt5_use_modules(${target} Widgets)
endif()

if(WIN32)
  set_target_properties(${target} PROPERTIES COMPILE_DEFINITIONS "CNOID_USE_GETTEXT_WRAPPER")
  target_link_libraries(${target} 
    optimized ${PROJECT_BINARY_DIR}/lib/Release/CnoidUtil.lib
    optimized ${PROJECT_BINARY_DIR}/lib/Release/CnoidBase.lib
    optimized ${PROJECT_BINARY_DIR}/lib/Release/CnoidBody.lib
    optimized ${PROJECT_BINARY_DIR}/${CNOID_PLUGIN_SUBDIR}/Release/CnoidBodyPlugin.lib
    optimized ${PROJECT_BINARY_DIR}/${CNOID_PLUGIN_SUBDIR}/Release/CnoidPoseSeqPlugin.lib
    debug ${PROJECT_BINARY_DIR}/lib/Debug/CnoidUtild.lib
    debug ${PROJECT_BINARY_DIR}/lib/Debug/CnoidBased.lib
    debug ${PROJECT_BINARY_DIR}/lib/Debug/CnoidBodyd.lib
    debug ${PROJECT_BINARY_DIR}/${CNOID_PLUGIN_SUBDIR}/Debug/CnoidBodyPlugind.lib
    debug ${PROJECT_BINARY_DIR}/${CNOID_PLUGIN_SUBDIR}/Debug/CnoidPoseSeqPlugind.lib
    ${QT_QTCORE_LIBRARY}
    ${QT_QTGUI_LIBRARY}
    )

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
    target_link_libraries(${target} CnoidPoseSeqPlugin)
  endif()
else()
  target_link_libraries(${target} ${PROJECT_BINARY_DIR}/${CNOID_PLUGIN_SUBDIR}/libCnoidPoseSeqPlugin.so)
endif()

apply_common_setting_for_plugin(${target} "${headers}")
