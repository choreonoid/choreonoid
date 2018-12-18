import WRSUtil
WRSUtil.loadProject(
    "SingleSceneView", "T6", [ "AGXSimulator", "AISTSimulator" ], "Quadcopter",
    enableMulticopterSimulation = True, enableVisionSimulation = True, remoteType = "ROS")
