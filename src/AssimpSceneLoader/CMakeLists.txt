if(ENABLE_ASSIMP)
  set(headers
    AssimpSceneLoader.h
    exportdecl.h
    )
  choreonoid_add_library(CnoidAssimpSceneLoader SHARED AssimpSceneLoader.cpp HEADERS ${headers})
  if(ASSIMP_USE_IMPORTED_LIBRARY)
    target_link_libraries(CnoidAssimpSceneLoader CnoidUtil assimp::assimp)
  else()
    target_link_libraries(CnoidAssimpSceneLoader CnoidUtil ${ASSIMP_LIBRARIES})
  endif()
endif()
