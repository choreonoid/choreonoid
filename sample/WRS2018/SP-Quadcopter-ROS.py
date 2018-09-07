import WRSUtil
WRSUtil.loadProject(
    "SingleSceneView", "SP", [ "AGXSimulator", "AISTSimulator" ], "Quadcopter",
    enableMulticopterSimulation = True, enableVisionSimulation = True, targetVisionSensors = "", remoteType = "ROS")
