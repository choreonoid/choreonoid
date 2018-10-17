if(NOT ENABLE_GUI)
  return()
endif()

option(BUILD_WRS2018_SAMPLES "Building WRS2018 samples" ON)
if(NOT BUILD_WRS2018_SAMPLES)
  return()
endif()

file(GLOB project_files "*.cnoid")
file(GLOB project_script_files "T?-*.py")
file(GLOB project_script_files "SP?-*.py")

install(FILES
  ${project_files}
  ${project_script_files}
  WRSUtil.py
  DESTINATION ${CNOID_SHARE_SUBDIR}/project/WRS2018)

add_subdirectory(src)
