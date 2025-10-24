option(BUILD_GL_VISION_SIMULATOR_PLUGIN "Building GLVisionSimulatorPlugin" ON)

if(NOT BUILD_GL_VISION_SIMULATOR_PLUGIN)
  return()
endif()

set(target CnoidGLVisionSimulatorPlugin)

set(sources
  GLVisionSimulatorPlugin.cpp
  GLVisionSimulatorItem.cpp
  GLVisionSensorScene.cpp
  GLVisionSensorRenderingScreen.cpp
  GLVisionSensorSimulator.cpp
  GLCameraSimulator.cpp
  GLRangeCameraSimulator.cpp
  GLRangeSensorSimulator.cpp
  GLFisheyeCameraSimulator.cpp
  FisheyeLensConverter.cpp
)

if(UNIX)
  list(APPEND sources ../GLSceneRenderer/gl_core_3_3.c)
endif()

set(headers
  GLVisionSimulatorItem.h
  GLVisionSensorSimulator.h
  exportdecl.h
  )

choreonoid_make_gettext_mo_files(${target} mofiles)
choreonoid_add_plugin(${target} ${sources} ${mofiles} ${RC_SRCS} HEADERS ${headers})
target_link_libraries(${target} PUBLIC CnoidBodyPlugin)

if(UNIX)
  # Link EGL library for Linux/Unix platforms (required by gl_core_3_3.c and GLVisionSensorRenderingScreen.cpp)
  target_link_libraries(${target} PRIVATE EGL)
endif()

if(ENABLE_PYTHON)
  add_subdirectory(pybind11)
endif()
