option(BUILD_BODY_MOTION_FILTER_PLUGIN "Building BodyMotionFilterPlugin" OFF)
mark_as_advanced(BUILD_BODY_MOTION_FILTER_PLUGIN)

if(NOT BUILD_BODY_MOTION_FILTER_PLUGIN)
  return()
elseif(NOT BUILD_POSE_SEQ_PLUGIN)
  message(FATAL_ERROR "BodyMotionFilterPlugins requires PoseSeqPlugin.")
endif()

# find_package(osqp REQUIRED)

set(target CnoidBodyMotionFilterPlugin)

set(sources
  BodyMotionFilterPlugin.cpp
  YawMomentCompensationFilter.cpp
  YawMomentCompensationDialog.cpp
  qld.c
  )

set(headers
  )

qt5_add_resources(RC_SRCS BodyMotionFilterPlugin.qrc)

make_gettext_mofiles(${target} mofiles)
choreonoid_add_plugin(${target} ${sources} ${mofiles} ${RC_SRCS} HEADERS ${headers})
target_link_libraries(${target} PUBLIC CnoidPoseSeqPlugin)
#target_link_libraries(${target} PUBLIC CnoidBodyPlugin PRIVATE osqp::osqpstatic)
