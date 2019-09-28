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

QT5_ADD_RESOURCES(RC_SRCS PoseSeqPlugin.qrc)

make_gettext_mofiles(${target} mofiles)
add_cnoid_plugin(${target} SHARED ${sources} ${headers} ${mofiles} ${RC_SRCS})
target_link_libraries(${target} CnoidUtil CnoidBody CnoidBodyPlugin)
apply_common_setting_for_plugin(${target} "${headers}")

if(ENABLE_PYTHON)
  add_subdirectory(pybind11)
endif()
