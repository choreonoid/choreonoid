if(ENABLE_ASSIMP AND ENABLE_GUI)
  choreonoid_add_plugin(CnoidAssimpPlugin AssimpPlugin.cpp)
  target_link_libraries(CnoidAssimpPlugin CnoidAssimpSceneLoader)
endif()
