
# @author Shin'ichiro Nakaoka

#set(CMAKE_BUILD_TYPE Debug)

set(sources
  AISTCollisionDetector.cpp
  ColdetModel.cpp
  ColdetModelPair.cpp
  StdCollisionPairInserter.cpp
  TriOverlap.cpp
  SSVTreeCollider.cpp
  DistFuncs.cpp
  Opcode/Ice/IceAABB.cpp
  Opcode/Ice/IceContainer.cpp
  Opcode/Ice/IceIndexedTriangle.cpp
  Opcode/Ice/IceMatrix3x3.cpp
  Opcode/Ice/IceMatrix4x4.cpp
  Opcode/Ice/IceRevisitedRadix.cpp
  Opcode/Ice/IceHPoint.cpp
  Opcode/Ice/IceRandom.cpp
  Opcode/Ice/IcePoint.cpp
  Opcode/Ice/IcePlane.cpp
  Opcode/OPC_AABBTree.cpp
  Opcode/OPC_BaseModel.cpp
  Opcode/OPC_Collider.cpp
  Opcode/OPC_Common.cpp
  Opcode/OPC_MeshInterface.cpp
  Opcode/OPC_Model.cpp
  Opcode/OPC_OptimizedTree.cpp
  Opcode/OPC_TreeBuilders.cpp
  Opcode/OPC_TreeCollider.cpp
  Opcode/OPC_VolumeCollider.cpp 
  Opcode/OPC_RayCollider.cpp
  Opcode/OPC_SphereCollider.cpp
  Opcode/OPC_Picking.cpp
  Opcode/OPC_PlanesCollider.cpp
  )

set(headers
  AISTCollisionDetector.h
  CollisionData.h
  ColdetModel.h
  ColdetModelPair.h
  CollisionPairInserter.h
  exportdecl.h
  )

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/Opcode)

if(MSVC)
  if(MSVC_VERSION GREATER 1600)
    add_definitions(-D_ALLOW_KEYWORD_MACROS)
  endif()
endif()

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
  set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl")
elseif(UNIX)
  # To hide OPCODE symbols which confict with ODE's OPCODE
  set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,-Bsymbolic")
 endif()

set(target CnoidAISTCollisionDetector)
choreonoid_add_library(${target} SHARED ${sources} HEADERS ${headers})
target_link_libraries(${target} CnoidUtil)
