if(ENABLE_ASSIMP)
  choreonoid_add_plugin(CnoidAssimpPlugin AssimpPlugin.cpp)
  target_link_libraries(CnoidAssimpPlugin PUBLIC CnoidAssimpSceneLoader)

  if(ENABLE_PYTHON)
    add_subdirectory(pybind11)
  endif()
endif()
