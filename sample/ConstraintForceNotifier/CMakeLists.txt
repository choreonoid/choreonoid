
#set(CMAKE_BUILD_TYPE Debug)

if(NOT ENABLE_GUI)
  return()
endif()

option(BUILD_CONSTRAINT_FORCE_NOTIFIER_SAMPLE "Building a sample for notifying constraint forces" OFF)
if(NOT BUILD_CONSTRAINT_FORCE_NOTIFIER_SAMPLE)
  return()
endif()

set(sources
  ConstraintForceNotifierSamplePlugin.cpp
)

set(target CnoidConstraintForceNotifierSamplePlugin)
add_cnoid_plugin(${target} SHARED ${sources})
target_link_libraries(${target} CnoidBodyPlugin)
apply_common_setting_for_plugin(${target})

configure_file(SR1Liftup-ConstraintForceNotifier.cnoid ${CNOID_SOURCE_SHARE_DIR}/project COPYONLY)
