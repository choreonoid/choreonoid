#set(CMAKE_BUILD_TYPE Debug)
#set_source_files_properties(JoystickLinux.cpp PROPERTIES COMPILE_FLAGS "-O0 -g")

configure_file(Config.h.in Config.h)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/Config.h DESTINATION ${CNOID_HEADER_SUBDIR}/src/Util/)

set(sources
  Referenced.cpp
  ConnectionSet.cpp
  Selection.cpp
  DataMap.cpp
  EigenUtil.cpp
  EigenArchive.cpp
  ValueTree.cpp
  YAMLReader.cpp
  YAMLWriter.cpp
  UTF8.cpp
  EasyScanner.cpp
  NullOut.cpp
  FileUtil.cpp
  ExecutablePath.cpp
  AbstractSeq.cpp
  Vector3Seq.cpp
  MultiSE3Seq.cpp
  MultiSE3MatrixSeq.cpp
  MultiValueSeq.cpp
  MultiVector3Seq.cpp
  GeneralSeqReader.cpp
  PlainSeqFileLoader.cpp
  Task.cpp
  AbstractTaskSequencer.cpp
  CollisionDetector.cpp
  RangeLimiter.cpp
  BoundingBox.cpp
  SceneGraph.cpp
  SceneDrawables.cpp
  SceneCameras.cpp
  SceneLights.cpp
  SceneEffects.cpp
  SceneRenderer.cpp
  SceneProvider.cpp
  SceneUtil.cpp
  MeshGenerator.cpp
  MeshFilter.cpp
  MeshExtractor.cpp
  SceneMarkers.cpp
  CoordinateAxesOverlay.cpp
  PolygonMeshTriangulator.cpp
  Image.cpp
  ImageIO.cpp
  ImageConverter.cpp
  ImageProvider.cpp
  PointSetUtil.cpp
  AbstractSceneLoader.cpp
  SceneLoader.cpp
  YAMLSceneLoader.cpp
  YAMLSceneReader.cpp
  STLSceneLoader.cpp
  VRML.cpp
  VRMLParser.cpp
  VRMLWriter.cpp
  VRMLToSGConverter.cpp
  VRMLSceneLoader.cpp
  GettextUtil.cpp
  ExtJoystick.cpp
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
  Referenced.h
  EasyScanner.h
  GaussianFilter.h
  UniformCubicBSpline.h
  IdPair.h
  Array2D.h
  Deque2D.h
  PolymorphicReferencedArray.h
  PolymorphicPointerArray.h
  PolymorphicFunctionSet.h
  MultiSE3Seq.h
  MultiSE3MatrixSeq.h
  MultiSeq.h
  MultiValueSeq.h
  MultiVector3Seq.h
  NullOut.h
  PlainSeqFileLoader.h
  RangeLimiter.h
  Seq.h
  AbstractSeq.h
  Timeval.h
  TimeMeasure.h
  Sleep.h
  Vector3Seq.h
  FileUtil.h
  ExecutablePath.h
  UTF8.h
  BoundingBox.h
  SceneGraph.h
  SceneDrawables.h
  SceneCameras.h
  SceneLights.h
  SceneEffects.h
  SceneRenderer.h
  SceneUtil.h
  AbstractSceneLoader.h
  SceneLoader.h
  MeshGenerator.h
  MeshFilter.h
  MeshExtractor.h
  SceneMarkers.h
  CoordinateAxesOverlay.h
  SceneProvider.h
  Collision.h
  CollisionDetector.h
  Triangulator.h
  PolygonMeshTriangulator.h
  PolyhedralRegion.h
  Image.h
  ImageIO.h
  ImageConverter.h
  ImageProvider.h
  PointSetUtil.h
  YAMLSceneLoader.h
  YAMLSceneReader.h
  VRML.h
  VRMLParser.h
  VRMLWriter.h
  VRMLToSGConverter.h
  Parser.h
  STLSceneLoader.h
  ValueTree.h
  ValueTreeUtil.h
  YAMLReader.h
  YAMLWriter.h
  EigenTypes.h
  EigenUtil.h
  EigenArchive.h
  Signal.h
  ConnectionSet.h
  GettextUtil.h
  Selection.h
  DataMap.h
  JoystickInterface.h
  Joystick.h
  ExtJoystick.h
  SharedJoystick.h
  Task.h
  AbstractTaskSequencer.h
  Exception.h
  exportdecl.h
  CnoidUtil.h
  )

set(target CnoidUtil)
make_gettext_mofiles(${target} mofiles)
add_cnoid_library(${target} SHARED ${sources} ${headers} ${mofiles})

if(UNIX)
  set(libraries 
    yaml ${PNG_LIBRARY} ${JPEG_LIBRARY}
    ${CMAKE_THREAD_LIBS_INIT}
    ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY}
    ${GETTEXT_LIBRARIES}
    m)

  if(CNOID_USE_BOOST_REGEX)
    set(libraries ${libraries} ${Boost_REGEX_LIBRARY})
  endif()

  if(APPLE)
    target_link_libraries(${target} ${libraries} "-framework IOKit -framework Foundation")
  else()
    target_link_libraries(${target} ${libraries})
  endif()
  
elseif(MSVC)
  set_target_properties(${target} PROPERTIES COMPILE_DEFINITIONS "YAML_DECLARE_STATIC")
  set(libraries libpng jpeg zlib winmm ${GETTEXT_LIBRARIES})
  if(USE_EXTERNAL_YAML)
    set(libraries ${libraries} optimized yaml debug yamld)
  else()
    set(libraries ${libraries} yaml)
  endif()
  target_link_libraries(${target} ${libraries})
endif()

apply_common_setting_for_library(${target} "${headers}")

if(ENABLE_PYTHON)
  if(USE_PYBIND11)
    add_subdirectory(pybind11)
  elseif(USE_BOOST_PYTHON)
    add_subdirectory(boostpython)
  endif()
endif()

add_subdirectory(lua)
