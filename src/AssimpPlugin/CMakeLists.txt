if(ENABLE_ASSIMP AND ENABLE_GUI)
  choreonoid_add_plugin(CnoidAssimpPlugin AssimpPlugin)
  target_link_libraries(CnoidAssimpPlugin CnoidBase CnoidAssimpSceneLoader)
endif()
