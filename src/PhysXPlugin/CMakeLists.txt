
# @author Shizuko Hattori

option(BUILD_PhysX_PLUGIN "Building PhysXPlugin" OFF)

if(NOT BUILD_PhysX_PLUGIN)
  return()
endif()

set(PhysX_DIR ${PhysX_DIR} CACHE PATH "set the top directory of PhysX")

if(NOT PhysX_DIR)
  message(FATAL_ERROR "Please specify the directory of PhysX to PhysX_DIR.")
endif()

if(PhysX_DIR)

  set(PhysX_INCLUDE_DIRS ${PhysX_DIR}/Include)

  if(UNIX)
    if(CMAKE_SIZEOF_VOID_P EQUAL 8)
      set(PhysX_LIBRARY_DIRS ${PhysX_DIR}/Lib/linux64 ${PhysX_DIR}/Bin/linux64)
      set(PhysX_LIBRARIES
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
    else()
      set(PhysX_LIBRARY_DIRS ${PhysX_DIR}/Lib/linux32)
      #add_definitions(-malign-double)
      set(PhysX_LIBRARIES
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
      set(PhysX_LIBRARY_DIRS ${PhysX_DIR}/Lib/win64)
      set(PhysX_LIBRARIES optimized PhysX3_x64 debug PhysX3DEBUG_x64
                          optimized PhysX3Common_x64 debug PhysX3CommonDEBUG_x64
                          optimized PhysXProfileSDK debug PhysXProfileSDKDEBUG
                          optimized PhysX3Extensions debug PhysX3ExtensionsDEBUG
                          optimized PhysX3Cooking_x64 debug PhysX3CookingDEBUG_x64 )
      if(INSTALL_RUNTIME_DEPENDENCIES)
        install(PROGRAMS ${PhysX_DIR}/Bin/win64/PhysX3_x64.dll DESTINATION bin CONFIGURATIONS Release RelWithDebInfo MinSizeRel)
        install(PROGRAMS ${PhysX_DIR}/Bin/win64/PhysX3CHECKED_x64.dll DESTINATION bin CONFIGURATIONS Debug)
        install(PROGRAMS ${PhysX_DIR}/Bin/win64/PhysX3Common_x64.dll DESTINATION bin CONFIGURATIONS Release RelWithDebInfo MinSizeRel)
        install(PROGRAMS ${PhysX_DIR}/Bin/win64/PhysX3CommonCHECKED_x64.dll DESTINATION bin CONFIGURATIONS Debug)
        install(PROGRAMS ${PhysX_DIR}/Bin/win64/nvToolsExt64_1.dll DESTINATION bin CONFIGURATIONS Release RelWithDebInfo MinSizeRel Debug)
        install(PROGRAMS ${PhysX_DIR}/Bin/win64/PhysX3Cooking_x64.dll DESTINATION bin CONFIGURATIONS Release RelWithDebInfo MinSizeRel)
        install(PROGRAMS ${PhysX_DIR}/Bin/win64/PhysX3CookingCHECKED_x64.dll DESTINATION bin CONFIGURATIONS Debug)
      endif()
    else()
      set(PhysX_LIBRARY_DIRS ${PhysX_DIR}/Lib/win32)
      set(PhysX_LIBRARIES optimized PhysX3_x86 debug PhysX3DEBUG_x86
                          optimized PhysX3Common_x86 debug PhysX3CommonDEBUG_x86
                          optimized PhysXProfileSDK debug PhysXProfileSDKDEBUG
                          optimized PhysX3Extensions debug PhysX3ExtensionsDEBUG
                          optimized PhysX3Cooking_x86 debug PhysX3CookingDEBUG_x86 )
      if(INSTALL_RUNTIME_DEPENDENCIES)
        install(PROGRAMS ${PhysX_DIR}/Bin/win32/PhysX3_x86.dll DESTINATION bin CONFIGURATIONS Release RelWithDebInfo MinSizeRel)
        install(PROGRAMS ${PhysX_DIR}/Bin/win32/PhysX3CHECKED_x86.dll DESTINATION bin CONFIGURATIONS Debug)
        install(PROGRAMS ${PhysX_DIR}/Bin/win32/PhysX3Common_x86.dll DESTINATION bin CONFIGURATIONS Release RelWithDebInfo MinSizeRel)
        install(PROGRAMS ${PhysX_DIR}/Bin/win32/PhysX3CommonCHECKED_x86.dll DESTINATION bin CONFIGURATIONS Debug)
        install(PROGRAMS ${PhysX_DIR}/Bin/win32/nvToolsExt32_1.dll DESTINATION bin CONFIGURATIONS Release RelWithDebInfo MinSizeRel Debug)
        install(PROGRAMS ${PhysX_DIR}/Bin/win32/PhysX3Cooking_x86.dll DESTINATION bin CONFIGURATIONS Release RelWithDebInfo MinSizeRel)
        install(PROGRAMS ${PhysX_DIR}/Bin/win32/PhysX3CookingCHECKED_x86.dll DESTINATION bin CONFIGURATIONS Debug)
      endif()
    endif()
  endif()
endif()

include_directories(${PhysX_INCLUDE_DIRS})
link_directories(${PhysX_LIBRARY_DIRS})

set(target CnoidPhysXPlugin)

set(sources
  PhysXPlugin.cpp
  PhysXSimulatorItem.cpp
  )

set(headers
  PhysXSimulatorItem.h
  )

make_gettext_mofiles(${target} mofiles)
add_cnoid_plugin(${target} SHARED ${sources} ${headers} ${mofiles})
target_link_libraries(${target} CnoidBodyPlugin ${PhysX_LIBRARIES})
apply_common_setting_for_plugin(${target} "${headers}")
