#set(CMAKE_BUILD_TYPE Debug)
#set_source_files_properties(SceneGraph.cpp PROPERTIES COMPILE_FLAGS "-O0 -g")

set(CNOID_ENABLE_GETTEXT ${CHOREONOID_ENABLE_GETTEXT})
set(CNOID_ENABLE_GPERFTOOLS_PROFILER ${ENABLE_GPERFTOOLS_PROFILER})
configure_file(Config.h.in Config.h)

set(sources
  Config.cpp
  Referenced.cpp
  CloneMap.cpp # This must be before any class using CloneMap::getFlagId.
  HierarchicalClassRegistry.cpp
  FileUtil.cpp
  ExecutablePath.cpp
  FilePathVariableProcessor.cpp
  GettextUtil.cpp
  UTF8.cpp
  NullOut.cpp
  StringUtil.cpp
  EasyScanner.cpp
  ValueTree.cpp
  YAMLReader.cpp
  YAMLWriter.cpp
  ArchiveSession.cpp
  EigenUtil.cpp
  EigenArchive.cpp
  Uuid.cpp
  GeneralId.cpp
  Selection.cpp
  DataMap.cpp
  AbstractSeq.cpp
  MultiValueSeq.cpp
  MultiSE3Seq.cpp
  MultiSE3MatrixSeq.cpp
  MultiVector3Seq.cpp
  Vector3Seq.cpp
  ReferencedObjectSeq.cpp
  GeneralSeqReader.cpp
  PlainSeqFileLoader.cpp
  RangeLimiter.cpp
  CoordinateFrame.cpp
  CoordinateFrameList.cpp
  PositionTag.cpp
  PositionTagGroup.cpp  
  BoundingBox.cpp
  SceneNodeClassRegistry.cpp # This must be before any scene graph classes
  PolymorphicSceneNodeFunctionSet.cpp
  SceneGraph.cpp
  SceneDrawables.cpp
  SceneCameras.cpp
  SceneLights.cpp
  SceneEffects.cpp
  SceneMarkers.cpp
  SceneRenderer.cpp
  CoordinateAxesOverlay.cpp # This must be after SceneRenderer.cpp
  SceneGraphOptimizer.cpp
  SceneUtil.cpp
  MeshGenerator.cpp
  MeshFilter.cpp
  MeshExtractor.cpp
  SceneNodeExtractor.cpp
  PolygonMeshTriangulator.cpp
  Image.cpp
  ImageIO.cpp
  ImageConverter.cpp
  PointSetUtil.cpp
  CollisionDetector.cpp
  AbstractSceneLoader.cpp
  SceneLoader.cpp
  AbstractSceneWriter.cpp
  StdSceneReader.cpp
  StdSceneLoader.cpp
  StdSceneWriter.cpp
  STLSceneLoader.cpp
  ObjSceneLoader.cpp
  ObjSceneWriter.cpp
  VRML.cpp
  VRMLParser.cpp
  VRMLWriter.cpp
  VRMLToSGConverter.cpp
  VRMLSceneLoader.cpp
  ExtJoystick.cpp
  Task.cpp
  AbstractTaskSequencer.cpp
  CnoidUtil.cpp # This file must be placed at the last position
  )


if(CMAKE_SYSTEM_NAME STREQUAL Linux)
  set(sources ${sources} JoystickLinux.cpp)
elseif(WIN32)
  set(sources ${sources} JoystickWindows.cpp)
elseif(APPLE)
  set(sources ${sources} JoystickOSX.cpp ysjoyreader-objc.m)
endif()

set(headers
  ${CMAKE_CURRENT_BINARY_DIR}/Config.h
  Referenced.h
  CloneMap.h
  ClonableReferenced.h
  HierarchicalClassRegistry.h
  PolymorphicFunctionSet.h
  PolymorphicReferencedArray.h
  PolymorphicPointerArray.h
  Signal.h
  ConnectionSet.h
  Exception.h
  Sleep.h
  ThreadPool.h
  Timeval.h
  TimeMeasure.h
  FileUtil.h
  ExecutablePath.h
  FilePathVariableProcessor.h
  GettextUtil.h
  UTF8.h
  NullOut.h
  StringUtil.h
  Tokenizer.h
  EasyScanner.h
  ValueTree.h
  ValueTreeUtil.h
  YAMLReader.h
  YAMLWriter.h
  ArchiveSession.h
  EigenTypes.h
  EigenUtil.h
  EigenArchive.h
  MathUtil.h
  Uuid.h
  GeneralId.h
  Selection.h
  DataMap.h
  IdPair.h
  Array2D.h
  Deque2D.h
  AbstractSeq.h
  Seq.h
  MultiSeq.h
  MultiValueSeq.h
  MultiSE3Seq.h
  MultiSE3MatrixSeq.h
  MultiVector3Seq.h
  Vector3Seq.h
  ReferencedObjectSeq.h
  PlainSeqFileLoader.h
  RangeLimiter.h
  GaussianFilter.h
  UniformCubicBSpline.h
  CoordinateFrame.h
  CoordinateFrameList.h
  PositionTag.h
  PositionTagGroup.h
  BoundingBox.h
  SceneNodeClassRegistry.h
  PolymorphicSceneNodeFunctionSet.h
  SceneUpdate.h
  SceneGraph.h
  SceneDrawables.h
  SceneCameras.h
  SceneLights.h
  SceneEffects.h
  SceneMarkers.h
  SceneRenderer.h
  CoordinateAxesOverlay.h
  SceneGraphOptimizer.h
  SceneUtil.h
  MeshGenerator.h
  MeshFilter.h
  MeshExtractor.h
  SceneNodeExtractor.h
  Triangulator.h
  PolygonMeshTriangulator.h
  PolyhedralRegion.h
  Image.h
  ImageIO.h
  ImageConverter.h
  PointSetUtil.h
  Collision.h
  CollisionDetector.h
  AbstractSceneLoader.h
  SceneLoader.h
  AbstractSceneWriter.h
  StdSceneReader.h
  StdSceneLoader.h
  StdSceneWriter.h
  STLSceneLoader.h
  ObjSceneLoader.h
  ObjSceneWriter.h
  SimpleScanner.h
  VRML.h
  VRMLParser.h
  VRMLWriter.h
  VRMLToSGConverter.h
  JoystickInterface.h
  Joystick.h
  ExtJoystick.h
  SharedJoystick.h
  Task.h
  AbstractTaskSequencer.h
  exportdecl.h
  CnoidUtil.h
  )

include_directories(
  ${LIBYAML_INCLUDE_DIRS} ${PNG_INCLUDE_DIRS} ${JPEG_INCLUDE_DIRS} ${LIBUUID_INCLUDE_DIRS})
link_directories(${LIBYAML_LIBRARY_DIRS})

include_directories(${PROJECT_SOURCE_DIR}/thirdparty/fast_float-0.8.0/include)

if(ENABLE_GPERFTOOLS_PROFILER)
  include_directories(${GPREFTOOLS_PROFILER_INCLUDE_DIRS})
  link_directories(${GPREFTOOLS_PROFILER_LIBRARY_DIRS})
  set(sources ${sources} ProfilerUtil.cpp)
endif()

set(target CnoidUtil)
make_gettext_mofiles(${target} mofiles)
choreonoid_add_library(${target} SHARED ${sources} ${mofiles} HEADERS ${headers})

if(UNIX)
  set(libraries 
    fmt::fmt
    ${LIBYAML_LIBRARIES} ${PNG_LIBRARIES} ${JPEG_LIBRARIES} ${LIBUUID_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
    ${Boost_SYSTEM_LIBRARY}
    ${GETTEXT_LIBRARIES}
    m dl
    )

  if(FILESYSTEM_LIBRARY)
    set(libraries ${libraries} ${FILESYSTEM_LIBRARY})
  endif()

  if(ENABLE_GPERFTOOLS_PROFILER)
    target_link_libraries(${target} ${GPREFTOOLS_PROFILER_LIBRARIES})
  endif()

  if(APPLE)
    target_link_libraries(${target} ${libraries} "-framework IOKit -framework Foundation")
  else()
    target_link_libraries(${target} ${libraries})
  endif()
  
elseif(MSVC)
  set_target_properties(${target} PROPERTIES COMPILE_DEFINITIONS "YAML_DECLARE_STATIC")
  set(libraries
    fmt::fmt
    ${LIBYAML_LIBRARIES} ${PNG_LIBRARIES} ${JPEG_LIBRARIES} ${GETTEXT_LIBRARIES}
    winmm Rpcrt4)
  find_file(XINPUT_DLL "XInput1_4.dll")
  if(XINPUT_DLL)
    target_link_libraries(${target} ${libraries} XInput)
    add_definitions(-DUSE_XINPUT)
  else()
    find_file(XINPUT_DLL "XInput9_1_0.dll")
    if(XINPUT_DLL)
      target_link_libraries(${target} ${libraries} XInput9_1_0)
      add_definitions(-DUSE_XINPUT)
    endif()
  endif()
endif()

if(ENABLE_PYTHON)
  add_subdirectory(pybind11)
endif()

add_subdirectory(lua)
