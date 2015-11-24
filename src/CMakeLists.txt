
add_subdirectory(Util)
add_subdirectory(Python)
add_subdirectory(AISTCollisionDetector)
add_subdirectory(Body)
add_subdirectory(Corba)

if(ENABLE_GUI)
  add_subdirectory(Base)
  add_subdirectory(PythonPlugin)
  add_subdirectory(BodyPlugin)
  add_subdirectory(PoseSeqPlugin)
  add_subdirectory(BalancerPlugin)
  add_subdirectory(SimpleControllerPlugin)
  add_subdirectory(ODEPlugin)
  add_subdirectory(BulletPlugin)
  add_subdirectory(PhysXPlugin)
  add_subdirectory(RokiPlugin)
  add_subdirectory(AgXPlugin)
  add_subdirectory(FCLPlugin)
  add_subdirectory(SceneEditPlugin)
  add_subdirectory(PCLPlugin)
  
  add_subdirectory(MediaPlugin)
  add_subdirectory(ScenarioPlugin)
  
  add_subdirectory(CorbaPlugin)
  add_subdirectory(OpenRTMPlugin)
  add_subdirectory(OpenHRPPlugin)
  
  add_subdirectory(RobotAccessPlugin)
  add_subdirectory(Hrpsys31Plugin)
  
  add_subdirectory(GrxUIPlugin)
  add_subdirectory(PythonSimScriptPlugin)

  add_subdirectory(Choreonoid)
endif()

