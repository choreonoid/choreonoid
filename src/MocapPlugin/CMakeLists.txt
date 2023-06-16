option(BUILD_MOCAP_PLUGIN "Building MocapPlugin" OFF)
if(NOT BUILD_MOCAP_PLUGIN)
  return()
endif()

set(target CnoidMocapPlugin)

set(sources
  MocapPlugin.cpp
  Bone.cpp
  Skeleton.cpp
  SkeletonMotion.cpp
  SkeletonMotionItem.cpp
  AMCLoader.cpp
  MarkerMotion.cpp
  MarkerMotionItem.cpp
  MocapMapping.cpp
  MocapMappingItem.cpp
  SkeletonToMarkerMotionConverter.cpp
  MocapConversionToBodyMotionDialog.cpp
  MocapConversionToBodyMotionPanel.cpp
  )

set(headers
  Bone.h
  Skeleton.h
  SkeletonMotion.h
  SkeletonMotionItem.h
  AMCLoader.h
  MarkerMotion.h
  MarkerMotionItem.h
  MocapMapping.h
  MocapMappingItem.h
  SkeletonToMarkerMotionConverter.h
  MocapConversionToBodyMotionPanel.h
  exportdecl.h
  )

choreonoid_make_gettext_mo_files(${target} mofiles)
choreonoid_add_plugin(${target} ${sources} ${mofiles} HEADERS ${headers})
target_link_libraries(${target} PUBLIC CnoidBodyPlugin)
