import WRSUtil
WRSUtil.loadProject(
    "SingleSceneView", "T1M", [ "AGXSimulator", "AISTSimulator" ], "Quadcopter",
    enableMulticopterSimulation = True, enableVisionSimulation = True, remoteType = "RTM")
