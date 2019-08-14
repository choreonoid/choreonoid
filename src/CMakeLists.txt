
add_subdirectory(Util)
add_subdirectory(AISTCollisionDetector)
add_subdirectory(AssimpSceneLoader)
add_subdirectory(Body)
add_subdirectory(Corba)
add_subdirectory(OpenRTM)

if(ENABLE_GUI)
  add_subdirectory(Base)
  add_subdirectory(AssimpPlugin)
  add_subdirectory(BodyPlugin)

  add_subdirectory(PoseSeqPlugin)
  add_subdirectory(BalancerPlugin)

  add_subdirectory(SceneEffectsPlugin)
  add_subdirectory(SceneEditPlugin)

  add_subdirectory(MediaPlugin)

  add_subdirectory(PythonPlugin)
  add_subdirectory(PythonSimScriptPlugin)

  add_subdirectory(LuaPlugin)

  add_subdirectory(ODEPlugin)
  add_subdirectory(AGXDynamicsPlugin)
  add_subdirectory(BulletPlugin)
  add_subdirectory(PhysXPlugin)
  add_subdirectory(SpringheadPlugin)
  add_subdirectory(RokiPlugin)
  add_subdirectory(MulticopterPlugin)

  add_subdirectory(CorbaPlugin)
  add_subdirectory(OpenRTMPlugin)
  add_subdirectory(TrafficControlPlugin)

  add_subdirectory(RobotAccessPlugin)
  add_subdirectory(Hrpsys31Plugin)

  add_subdirectory(FCLPlugin)
  add_subdirectory(SDFPlugin)
  add_subdirectory(PCLPlugin)

  add_subdirectory(CompetitionPlugin)

  add_subdirectory(Choreonoid)
endif()
