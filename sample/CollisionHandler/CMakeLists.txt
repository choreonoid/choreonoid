
#set(CMAKE_BUILD_TYPE Debug)

if(NOT ENABLE_GUI)
  return()
endif()

option(BUILD_COLLISION_HANDLER_SAMPLE "Building a collision handler sample" OFF)
if(NOT BUILD_COLLISION_HANDLER_SAMPLE)
  return()
endif()

set(sources
  SpringDamperContactPlugin.cpp
)

set(target CnoidSpringDamperContactPlugin)
add_cnoid_plugin(${target} SHARED ${sources})
target_link_libraries(${target} CnoidBodyPlugin)
apply_common_setting_for_plugin(${target})

configure_file(SR1Liftup-SpringDamperContact.cnoid ${CNOID_SOURCE_SHARE_DIR}/project COPYONLY)
configure_file(SR1-SpringDamperContact.py ${CNOID_SOURCE_SHARE_DIR}/script COPYONLY)
