
#set(CMAKE_BUILD_TYPE Debug)
#set_source_files_properties(JoystickLinux.cpp PROPERTIES COMPILE_FLAGS "-O0 -g")

configure_file(Config.h.in ${CMAKE_CURRENT_SOURCE_DIR}/Config.h)

set(sources
  ConnectionSet.cpp
  Selection.cpp
  DataMap.cpp
  EigenUtil.cpp
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
  MultiAffine3Seq.cpp
  MultiValueSeq.cpp
  MultiVector3Seq.cpp
  PlainSeqFormatLoader.cpp
  Task.cpp
  CollisionDetector.cpp
  RangeLimiter.cpp
  BoundingBox.cpp
  SceneGraph.cpp
  SceneVisitor.cpp
  SceneProvider.cpp
  SceneUtil.cpp
  MeshGenerator.cpp
  MeshNormalGenerator.cpp
  MeshExtractor.cpp
  SceneMarker.cpp
  PolygonMeshTriangulator.cpp
  Image.cpp
  ImageIO.cpp
  ImageConverter.cpp
  PointSetUtil.cpp
  VRML.cpp
  VRMLParser.cpp
  VRMLWriter.cpp
  VRMLToSGConverter.cpp
  DaeParser.cpp
  STLSceneLoader.cpp
  GettextWrapper.cpp
  GettextUtil.cpp
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
  EasyScanner.h
  GaussianFilter.h
  UniformCubicBSpline.h
  IdPair.h
  Array2D.h
  Deque2D.h
  PolymorphicReferencedArray.h
  PolymorphicPointerArray.h
  MultiSE3Seq.h
  MultiAffine3Seq.h
  MultiSeq.h
  MultiValueSeq.h
  MultiVector3Seq.h
  NullOut.h
  PlainSeqFormatLoader.h
  RangeLimiter.h
  Referenced.h
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
  SceneShape.h
  SceneCamera.h
  SceneLight.h
  SceneVisitor.h
  SceneRenderer.h
  SceneUtil.h
  MeshGenerator.h
  MeshNormalGenerator.h
  MeshExtractor.h
  SceneMarker.h
  SceneProvider.h
  CollisionDetector.h
  Triangulator.h
  PolygonMeshTriangulator.h
  Image.h
  ImageIO.h
  ImageConverter.h
  PointSetUtil.h
  VRML.h
  VRMLParser.h
  VRMLWriter.h
  VRMLToSGConverter.h
  Parser.h
  DaeNode.h
  DaeParser.h
  AbstractSceneLoader.h
  STLSceneLoader.h
  ValueTree.h
  ValueTreeUtil.h
  YAMLReader.h
  YAMLWriter.h
  EigenTypes.h
  EigenUtil.h
  EigenArchive.h
  Signal.h
  SignalTemplate.h
  ConnectionSet.h
  GettextUtil.h
  Selection.h
  DataMap.h
  Joystick.h
  Task.h
  exportdecl.h
  Config.h
  )

include_directories(${IRRXML_INCLUDE_DIRS})

set(target CnoidUtil)
make_gettext_mofiles(${target} mofiles)
add_cnoid_library(${target} SHARED ${sources} ${headers} ${mofiles})

if(UNIX)
  set(libraries 
    yaml irrXML ${PNG_LIBRARY} ${JPEG_LIBRARY}
    ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY} ${Boost_THREAD_LIBRARY}
    ${GETTEXT_LIBRARIES}
    m)

  if(APPLE)
    target_link_libraries(${target} ${libraries} "-framework IOKit -framework Foundation")
  else()
    target_link_libraries(${target} ${libraries})
  endif()
  
elseif(MSVC)
  set_target_properties(${target} PROPERTIES COMPILE_DEFINITIONS "YAML_DECLARE_STATIC")
  set(libraries libpng jpeg zlib irrXML winmm ${GETTEXT_LIBRARIES} ${IRRXML_LIBRARY})
  if(USE_EXTERNAL_YAML)
    set(libraries ${libraries} optimized yaml debug yamld)
  else()
    set(libraries ${libraries} yaml)
  endif()
  target_link_libraries(${target} ${libraries})
endif()


apply_common_setting_for_library(${target} "${headers}")


# Adding some headers with old filenames for the backward compatibility
if(CNOID_ENABLE_BACKWARD_COMPATIBILITY)
  if(UNIX AND NOT APPLE)
    make_header_public(VRML.h VRMLNodes)
    make_header_public(VRML.h VrmlNodes)
    make_header_public(VRMLParser.h VrmlParser)
    make_header_public(VRMLWriter.h VrmlWriter)
    make_header_public(YAMLReader.h YamlReader)
    make_header_public(YAMLWriter.h YamlWriter)
  endif()
  make_header_public(ValueTree.h YamlNodes)
  make_header_public(EigenArchive.h EigenYaml)
endif()


if(ENABLE_PYTHON)
  add_subdirectory(python)
endif()
