option(BUILD_PCL_PLUGIN "Building PCLPlugin" OFF)

if(NOT BUILD_PCL_PLUGIN)
  return()
endif()

# Install PCL packages by the following command in Ubuntu
# sudo apt install libpcl-dev

find_package(PCL REQUIRED common features io registration surface)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(sources
  PCLPlugin.cpp
  PointCloudUtil.cpp
  PointSetToMeshConversionDialog.cpp
  )

set(headers
  PointCloudUtil.h
  exportdecl.h
)

set(target CnoidPCLPlugin)

choreonoid_make_gettext_mo_files(${target} mofiles)
choreonoid_add_plugin(${target} ${sources} ${mofiles} HEADERS ${headers})
target_link_libraries(
  ${target} PUBLIC
  CnoidBodyPlugin
  ${PCL_COMMON_LIBRARIES} ${PCL_FEATURES_LIBRARIES} ${PCL_REGISTRATION_LIBRARIES} ${PCL_SURFACE_LIBRARIES})
