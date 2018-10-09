import WRSUtil
WRSUtil.loadProject(
    "SingleSceneView", "SP", [ "AGXSimulator", "AISTSimulator" ], "Quadcopter",
    enableMulticopterSimulation = True, enableVisionSimulation = True, remoteType = "RTM")
