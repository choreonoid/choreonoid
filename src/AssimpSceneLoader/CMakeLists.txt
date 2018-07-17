if(ENABLE_ASSIMP)
  include_directories(${ASSIMP_INCLUDE_DIRS})
  link_directories(${ASSIMP_LIBRARY_DIRS})
  add_cnoid_library(CnoidAssimpSceneLoader SHARED AssimpSceneLoader.cpp)
  target_link_libraries(CnoidAssimpSceneLoader CnoidUtil ${ASSIMP_LIBRARIES})
  apply_common_setting_for_library(CnoidAssimpSceneLoader)
endif()

