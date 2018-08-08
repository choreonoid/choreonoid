if(NOT ENABLE_GUI)
  return()
endif()

option(BUILD_CONTACT_FORCE_EXTRACTION_SAMPLE "Building a sample for extracting contact forces" OFF)
if(NOT BUILD_CONTACT_FORCE_EXTRACTION_SAMPLE)
  return()
endif()

set(target CnoidContactForceExtractionSamplePlugin)
add_cnoid_plugin(${target} SHARED ContactForceExtractionSamplePlugin.cpp)
target_link_libraries(${target} CnoidBodyPlugin)
apply_common_setting_for_plugin(${target})

install(FILES SR1Liftup-ContactForceExtraction.cnoid DESTINATION ${CNOID_SHARE_SUBDIR}/project)
