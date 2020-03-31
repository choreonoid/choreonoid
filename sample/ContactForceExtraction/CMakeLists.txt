if(NOT ENABLE_GUI)
  return()
endif()

option(BUILD_CONTACT_FORCE_EXTRACTION_SAMPLE "Building a sample for extracting contact forces" OFF)
if(NOT BUILD_CONTACT_FORCE_EXTRACTION_SAMPLE)
  return()
endif()

set(target CnoidContactForceExtractionSamplePlugin)
choreonoid_add_plugin(${target} SHARED ContactForceExtractionSamplePlugin.cpp)
target_link_libraries(${target} CnoidBodyPlugin)

install(FILES SR1Liftup-ContactForceExtraction.cnoid DESTINATION ${CNOID_SHARE_SUBDIR}/project)
