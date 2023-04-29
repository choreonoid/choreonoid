option(BUILD_BALANCER_PLUGIN "Building BalancerPlugin" OFF)
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

choreonoid_make_gettext_mo_files(${target} mofiles)
choreonoid_add_plugin(${target} ${sources} ${mofiles})

add_dependencies(${target} CnoidPoseSeqPlugin)

if(WIN32)
  set_target_properties(${target} PROPERTIES COMPILE_DEFINITIONS "CNOID_USE_GETTEXT_WRAPPER")
endif()

target_link_libraries(${target} PUBLIC CnoidPoseSeqPlugin)
