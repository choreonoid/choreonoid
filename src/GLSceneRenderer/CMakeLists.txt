set(sources
  GLSceneRenderer.cpp
  GL1SceneRenderer.cpp
  GLSLProgram.cpp
  ShaderPrograms.cpp
  GLSLSceneRenderer.cpp
  gl_core_3_3.c
  #gl_core_4_0.c
  #gl_core_4_4.c
  gl_1_5.c
  )

set(headers
  GLSceneRenderer.h
  GL1SceneRenderer.h
  GLSLProgram.h
  ShaderPrograms.h
  GLSLSceneRenderer.h
  exportdecl.h
  )

set(target CnoidGLSceneRenderer)

QT5_ADD_RESOURCES(RC_SRCS GLSceneRenderer.qrc)

make_gettext_mofiles(${target} mofiles)

set(libraries CnoidUtil Qt5::Core)

if(MSVC)
  set(libraries ${libraries} general glu32 opengl32)
else()
  set(libraries ${libraries} OpenGL::GLU)
endif()

if(CNOID_ENABLE_FREE_TYPE)
  set(sources ${sources} GLFreeType.cpp)
  if(MSVC)
    set(libraries ${libraries} Freetype::Freetype)
  else()
    set(libraries ${libraries} PkgConfig::FreeType2)
  endif()
endif()

choreonoid_add_library(${target} SHARED ${sources} ${mofiles} ${RC_SRCS} HEADERS ${headers})
target_link_libraries(${target} ${libraries})
