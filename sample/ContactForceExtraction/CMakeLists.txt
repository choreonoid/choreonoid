
#set(CMAKE_BUILD_TYPE Debug)

if(NOT ENABLE_GUI)
  return()
endif()

option(BUILD_CONTACT_FORCE_EXTRACTION_SAMPLE "Building a sample for extracting contact forces" OFF)
if(NOT BUILD_CONTACT_FORCE_EXTRACTION_SAMPLE)
  return()
endif()

set(sources
  ContactForceExtractionSamplePlugin.cpp
)

set(target CnoidContactForceExtractionSamplePlugin)
add_cnoid_plugin(${target} SHARED ${sources})
target_link_libraries(${target} CnoidBodyPlugin)
apply_common_setting_for_plugin(${target})

configure_file(SR1Liftup-ContactForceExtraction.cnoid ${CNOID_SOURCE_SHARE_DIR}/project COPYONLY)
