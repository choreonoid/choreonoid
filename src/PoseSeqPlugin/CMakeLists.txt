option(BUILD_POSE_SEQ_PLUGIN "Building PoseSeqPlugin" OFF)

if(NOT BUILD_POSE_SEQ_PLUGIN)
  return()
endif()

set(target CnoidPoseSeqPlugin)

set(sources
  PoseSeqPlugin.cpp
  AbstractPose.cpp
  BodyKeyPose.cpp
  SequentialPose.cpp
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
  BodyKeyPoseSelectionDialog.cpp
  FcpFileLoader.cpp
  FootFixFilter.cpp
)

set(headers
  AbstractPose.h
  BodyKeyPose.h
  SequentialPose.h
  PoseSeq.h
  PoseSeqInterpolator.h
  PoseFilters.h
  PoseSeqItem.h
  PoseRollView.h
  BodyMotionGenerationBar.h
  exportdecl.h
  )

option(ENABLE_HUMANOID_POSE_FETCH_VIEW OFF)
mark_as_advanced(ENABLE_HUMANOID_POSE_FETCH_VIEW)
if(ENABLE_HUMANOID_POSE_FETCH_VIEW)
  add_definitions(-DCNOID_ENABLE_HUMANOID_POSE_FETCH_VIEW)
set(sources ${sources} HumanoidPoseFetchView.cpp)
endif()

qt5_add_resources(RC_SRCS PoseSeqPlugin.qrc)

choreonoid_make_gettext_mo_files(${target} mofiles)
choreonoid_add_plugin(${target} ${sources} ${mofiles} ${RC_SRCS} HEADERS ${headers})
target_link_libraries(${target} PUBLIC CnoidBodyPlugin)

if(ENABLE_PYTHON)
  add_subdirectory(pybind11)
endif()
