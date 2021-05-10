# @author Shizuko Hattori

option(BUILD_PHYSX_PLUGIN "Building PhysXPlugin" OFF)
mark_as_advanced(BUILD_PHYSX_PLUGIN)

if(NOT BUILD_PHYSX_PLUGIN)
  return()
endif()

set(PHYSX_DIR ${PHYSX_DIR} CACHE PATH "set the top directory of PhysX")

if(NOT PHYSX_DIR)
  message(FATAL_ERROR "Please specify the directory of PhysX to PHYSX_DIR.")
endif()

if(PHYSX_DIR)

  set(PHYSX_VERSION 3.4)
  if(EXISTS ${PHYSX_DIR}/version-PhysX.txt)
    file(READ ${PHYSX_DIR}/version-PhysX.txt versiontext)
    if( ${versiontext} MATCHES "PhysX-3\\.3.*")
      set(PHYSX_VERSION 3.3 )
    endif()
  endif()
  
  if(${PHYSX_VERSION} VERSION_EQUAL 3.3)
    set(PHYSX_INCLUDE_DIRS ${PHYSX_DIR}/Include)
  elseif(${PHYSX_VERSION} VERSION_EQUAL 3.4)
    set(PHYSX_INCLUDE_DIRS ${PHYSX_DIR}/Include ${PHYSX_DIR}/../PxShared/include)
  endif()
  
  if(UNIX)
    if(CMAKE_SIZEOF_VOID_P EQUAL 8)
      if(${PHYSX_VERSION} VERSION_EQUAL 3.3)
        set(PHYSX_LIBRARY_DIRS ${PHYSX_DIR}/Lib/linux64 ${PHYSX_DIR}/Bin/linux64)
        set(PHYSX_LIBRARIES
        PvdRuntime
        PhysX3_x64
        SimulationController
        SceneQuery
        LowLevel
        LowLevelCloth
        PhysX3Vehicle
        PhysX3Cooking_x64
        PhysX3Extensions
        PhysX3CharacterKinematic_x64
        PhysXProfileSDK
        PxTask
        PhysX3Common_x64
      )
      elseif(${PHYSX_VERSION} VERSION_EQUAL 3.4)
        set(PHYSX_LIBRARY_DIRS ${PHYSX_DIR}/Lib/linux64 ${PHYSX_DIR}/Bin/linux64 ${PHYSX_DIR}/../PxShared/Lib/linux64 ${PHYSX_DIR}/../PxShared/Bin/linux64)
        set(PHYSX_LIBRARIES
        PhysX3_x64
        SimulationController
        SceneQuery
        LowLevel
        LowLevelCloth
        PhysX3Vehicle
        PhysX3Cooking_x64
        PhysX3Extensions
        PhysX3CharacterKinematic_x64
        PhysX3Common_x64
      )
      endif()
    else()
      set(PHYSX_LIBRARY_DIRS ${PHYSX_DIR}/Lib/linux32)
      #add_definitions(-malign-double)
      set(PHYSX_LIBRARIES
        PvdRuntime
        PhysX3_x86
        SimulationController
        SceneQuery
        LowLevel
        LowLevelCloth
        PhysX3Vehicle
        PhysX3Cooking_x86
        PhysX3Extensions
        PhysX3CharacterKinematic_x86
        PhysXProfileSDK
        PxTask
        PhysX3Common_x86
      )
    endif()
  elseif(MSVC)
    if(CMAKE_CL_64)
      if(${PHYSX_VERSION} VERSION_EQUAL 3.3)
        set(PHYSX_LIBRARY_DIRS ${PHYSX_DIR}/Lib/vc14win64)
        set(PHYSX_LIBRARIES optimized PhysX3_x64 debug PhysX3DEBUG_x64
                            optimized PhysX3Common_x64 debug PhysX3CommonDEBUG_x64
                            optimized PhysXProfileSDK debug PhysXProfileSDKDEBUG
                            optimized PhysX3Extensions debug PhysX3ExtensionsDEBUG
                            optimized PhysX3Cooking_x64 debug PhysX3CookingDEBUG_x64 )
        if(INSTALL_RUNTIME_DEPENDENCIES)
          install(PROGRAMS ${PHYSX_DIR}/Bin/vc14win64/PhysX3_x64.dll DESTINATION bin CONFIGURATIONS Release RelWithDebInfo MinSizeRel)
          install(PROGRAMS ${PHYSX_DIR}/Bin/vc14win64/PhysX3CHECKED_x64.dll DESTINATION bin CONFIGURATIONS Debug)
          install(PROGRAMS ${PHYSX_DIR}/Bin/vc14win64/PhysX3Common_x64.dll DESTINATION bin CONFIGURATIONS Release RelWithDebInfo MinSizeRel)
          install(PROGRAMS ${PHYSX_DIR}/Bin/vc14win64/PhysX3CommonCHECKED_x64.dll DESTINATION bin CONFIGURATIONS Debug)
          install(PROGRAMS ${PHYSX_DIR}/Bin/vc14win64/nvToolsExt64_1.dll DESTINATION bin CONFIGURATIONS Release RelWithDebInfo MinSizeRel Debug)
          install(PROGRAMS ${PHYSX_DIR}/Bin/vc14win64/PhysX3Cooking_x64.dll DESTINATION bin CONFIGURATIONS Release RelWithDebInfo MinSizeRel)
          install(PROGRAMS ${PHYSX_DIR}/Bin/vc14win64/PhysX3CookingCHECKED_x64.dll DESTINATION bin CONFIGURATIONS Debug)
        endif()
      elseif(${PHYSX_VERSION} VERSION_EQUAL 3.4)
        set(PHYSX_LIBRARY_DIRS ${PHYSX_DIR}/Lib/vc14win64 ${PHYSX_DIR}/../PxShared/Lib/vc14win64)
        set(PHYSX_LIBRARIES optimized PhysX3_x64 debug PhysX3DEBUG_x64
                            optimized PhysX3Common_x64 debug PhysX3CommonDEBUG_x64
                            optimized PhysX3Extensions debug PhysX3ExtensionsDEBUG
                            optimized PxFoundation_x64 debug PxFoundationDEBUG_x64
                            optimized PhysX3Cooking_x64 debug PhysX3CookingDEBUG_x64 )
        if(INSTALL_RUNTIME_DEPENDENCIES)
          install(PROGRAMS ${PHYSX_DIR}/Bin/vc14win64/PhysX3_x64.dll DESTINATION bin CONFIGURATIONS Release RelWithDebInfo MinSizeRel)
          install(PROGRAMS ${PHYSX_DIR}/Bin/vc14win64/PhysX3CHECKED_x64.dll DESTINATION bin CONFIGURATIONS Debug)
          install(PROGRAMS ${PHYSX_DIR}/Bin/vc14win64/PhysX3Common_x64.dll DESTINATION bin CONFIGURATIONS Release RelWithDebInfo MinSizeRel)
          install(PROGRAMS ${PHYSX_DIR}/Bin/vc14win64/PhysX3CommonCHECKED_x64.dll DESTINATION bin CONFIGURATIONS Debug)
          install(PROGRAMS ${PHYSX_DIR}/Bin/vc14win64/nvToolsExt64_1.dll DESTINATION bin CONFIGURATIONS Release RelWithDebInfo MinSizeRel Debug)
          install(PROGRAMS ${PHYSX_DIR}/Bin/vc14win64/PhysX3Cooking_x64.dll DESTINATION bin CONFIGURATIONS Release RelWithDebInfo MinSizeRel)
          install(PROGRAMS ${PHYSX_DIR}/Bin/vc14win64/PhysX3CookingCHECKED_x64.dll DESTINATION bin CONFIGURATIONS Debug)
          install(PROGRAMS ${PHYSX_DIR}/../PxShared/bin/vc14win64/PxFoundation_x64.dll DESTINATION bin CONFIGURATIONS Release RelWithDebInfo MinSizeRel)
        endif()
      endif()
    else()
      set(PHYSX_LIBRARY_DIRS ${PHYSX_DIR}/Lib/win32)
      set(PHYSX_LIBRARIES optimized PhysX3_x86 debug PhysX3DEBUG_x86
                          optimized PhysX3Common_x86 debug PhysX3CommonDEBUG_x86
                          optimized PhysXProfileSDK debug PhysXProfileSDKDEBUG
                          optimized PhysX3Extensions debug PhysX3ExtensionsDEBUG
                          optimized PhysX3Cooking_x86 debug PhysX3CookingDEBUG_x86 )
      if(INSTALL_RUNTIME_DEPENDENCIES)
        install(PROGRAMS ${PHYSX_DIR}/Bin/win32/PhysX3_x86.dll DESTINATION bin CONFIGURATIONS Release RelWithDebInfo MinSizeRel)
        install(PROGRAMS ${PHYSX_DIR}/Bin/win32/PhysX3CHECKED_x86.dll DESTINATION bin CONFIGURATIONS Debug)
        install(PROGRAMS ${PHYSX_DIR}/Bin/win32/PhysX3Common_x86.dll DESTINATION bin CONFIGURATIONS Release RelWithDebInfo MinSizeRel)
        install(PROGRAMS ${PHYSX_DIR}/Bin/win32/PhysX3CommonCHECKED_x86.dll DESTINATION bin CONFIGURATIONS Debug)
        install(PROGRAMS ${PHYSX_DIR}/Bin/win32/nvToolsExt32_1.dll DESTINATION bin CONFIGURATIONS Release RelWithDebInfo MinSizeRel Debug)
        install(PROGRAMS ${PHYSX_DIR}/Bin/win32/PhysX3Cooking_x86.dll DESTINATION bin CONFIGURATIONS Release RelWithDebInfo MinSizeRel)
        install(PROGRAMS ${PHYSX_DIR}/Bin/win32/PhysX3CookingCHECKED_x86.dll DESTINATION bin CONFIGURATIONS Debug)
      endif()
    endif()
  endif()
endif()

include_directories(${PHYSX_INCLUDE_DIRS})
link_directories(${PHYSX_LIBRARY_DIRS})

set(target CnoidPhysXPlugin)

set(sources
  PhysXPlugin.cpp
  PhysXSimulatorItem.cpp
  )

set(headers
  PhysXSimulatorItem.h
  )

make_gettext_mofiles(${target} mofiles)
choreonoid_add_plugin(${target} ${sources} ${mofiles} HEADERS ${headers})
target_link_libraries(${target} CnoidBodyPlugin ${PHYSX_LIBRARIES})
