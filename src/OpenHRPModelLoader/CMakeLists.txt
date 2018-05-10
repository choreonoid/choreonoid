# @author Shizuko Hattori

option(BUILD_OPENHRP_MODEL_LOADER "Building OpenHRPModelLoader" OFF)
if(NOT BUILD_OPENHRP_MODEL_LOADER)
  return()
elseif(NOT BUILD_OPENHRP_PLUGIN) 
  message(FATAL_ERROR "OpenHRPModelLoader requires OpenHRP Plugin.")
endif()

set(target openhrpModelLoader)

set(sources
  ShapeSetInfo_impl.cpp
  SceneInfo_impl.cpp
  BodyInfo_impl.cpp
  ModelLoader_impl.cpp
  ModelLoaderUtil.cpp
  ModelLoaderServer.cpp
  ../OpenHRPPlugin/corba/OpenHRP/3.1/ModelLoaderSk.cpp
  ../OpenHRPPlugin/corba/OpenHRP/3.1/OpenHRPCommonSk.cpp
)

add_cnoid_executable(${target} ${sources} ${idl_cpp_files} )
target_link_libraries(${target} CnoidCorba CnoidBody ${OMNIORB_LIBRARIES} )
