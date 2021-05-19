option(BUILD_PCL_PLUGIN "Building PCLPlugin" OFF)
mark_as_advanced(BUILD_PCL_PLUGIN)

if(NOT BUILD_PCL_PLUGIN)
  return()
endif()

find_package(PCL REQUIRED common io surface features)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(sources
  PCLPlugin.cpp
  PointCloudUtil.cpp
  )

set(headers
  PointCloudUtil.h
  exportdecl.h
)

set(target CnoidPCLPlugin)

make_gettext_mofiles(${target} mofiles)
choreonoid_add_plugin(${target} ${sources} ${mofiles} HEADERS ${headers})
target_link_libraries(${target} ${PCL_COMMON_LIBRARIES} ${PCL_SURFACE_LIBRARIES} ${PCL_FEATURES_LIBRARIES} pcl_io)
