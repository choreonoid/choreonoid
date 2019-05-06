# @author Shizuko Hattori

option(BUILD_OPENHRP_MODEL_LOADER "Building OpenHRPModelLoader" OFF)
if(NOT BUILD_OPENHRP_MODEL_LOADER)
  return()
endif()

set(target cnoid-openhrp-model-loader)

set(sources
  ShapeSetInfo_impl.cpp
  SceneInfo_impl.cpp
  BodyInfo_impl.cpp
  ModelLoader_impl.cpp
  ModelLoaderUtil.cpp
  ModelLoaderServer.cpp
)

set(idls
  OpenHRPCommon
  ModelLoader
  ViewSimulator
  World
)

set(idl_cpp_files "")
set(idl_h_files "")
idl_compile_cpp(idl_cpp_files idl_h_files corba/OpenHRPModelLoader ${idls})
      
add_cnoid_executable(${target} ${sources} ${idl_cpp_files})
target_link_libraries(${target} CnoidCorba CnoidBody ${OMNIDYNAMIC_LIBRARIES})
