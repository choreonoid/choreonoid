import WRSUtil
WRSUtil.loadProject(
    "SingleSceneView", "T2", [ "AGXSimulator", "AISTSimulator" ], "Quadcopter",
    enableMulticopterSimulation = True, enableVisionSimulation = True, remoteType = "ROS")
