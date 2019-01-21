if(NOT ENABLE_GUI)
  return()
endif()

option(BUILD_WRS2018 "Building the WRS2018 files" OFF)
if(NOT BUILD_WRS2018)
  return()
endif()

if(UNIX)
  add_custom_target(
    symlink_to_WRS2018 ALL
    COMMAND ${CMAKE_COMMAND} -E create_symlink
    ${CMAKE_CURRENT_SOURCE_DIR} ${CNOID_BINARY_SHARE_DIR}/WRS2018)
endif()

install(DIRECTORY
  model
  project
  script
  test
  DESTINATION ${CNOID_SHARE_SUBDIR}/WRS2018)

add_subdirectory(controller)
