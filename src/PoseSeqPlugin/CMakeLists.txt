# @author Shin'ichiro Nakaoka

# set(CMAKE_BUILD_TYPE Debug)

option(BUILD_POSE_SEQ_PLUGIN "Building PoseSeqPlugin" ON)

if(NOT BUILD_POSE_SEQ_PLUGIN)
  return()
endif()

set(target CnoidPoseSeqPlugin)

set(sources
  PoseSeqPlugin.cpp
  Pose.cpp
  PoseSeq.cpp
  PoseSeqInterpolator.cpp
  PronunSymbol.cpp
  PoseFilters.cpp
  LipSyncTranslator.cpp
  PoseSeqItem.cpp
  PoseSeqEngine.cpp
  PoseSeqViewBase.cpp
  PoseRollView.cpp
  BodyMotionGenerationBar.cpp
  FcpFileLoader.cpp
)

set(headers
  Pose.h
  PoseSeq.h
  PoseSeqInterpolator.h
  PoseFilters.h
  PoseSeqItem.h
  BodyMotionGenerationBar.h
  exportdecl.h
  )

qt5_add_resources(RC_SRCS PoseSeqPlugin.qrc)

choreonoid_make_gettext_mo_files(${target} mofiles)
choreonoid_add_plugin(${target} ${sources} ${mofiles} ${RC_SRCS} HEADERS ${headers})
target_link_libraries(${target} CnoidBodyPlugin)

if(ENABLE_PYTHON)
  add_subdirectory(pybind11)
endif()
