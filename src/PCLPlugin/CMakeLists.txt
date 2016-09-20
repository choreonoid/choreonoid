
option(BUILD_PCL_PLUGIN "Building PCLPlugin" OFF)

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
add_cnoid_plugin(${target} SHARED ${sources} ${headers} ${mofiles})
target_link_libraries(${target} CnoidBase ${PCL_COMMON_LIBRARIES} ${PCL_SURFACE_LIBRARIES} ${PCL_FEATURES_LIBRARIES} pcl_io)
apply_common_setting_for_plugin(${target} "${headers}")
