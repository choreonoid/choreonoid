import WRSUtil
WRSUtil.loadProject(
    "SingleSceneView", "T3", [ "AGXSimulator", "AISTSimulator" ], "Quadcopter",
    enableMulticopterSimulation = True, enableVisionSimulation = True, remoteType = "RTM")
