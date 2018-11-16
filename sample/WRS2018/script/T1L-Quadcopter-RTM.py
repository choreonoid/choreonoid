import WRSUtil
WRSUtil.loadProject(
    "SingleSceneView", "T1L", [ "AGXSimulator", "AISTSimulator" ], "Quadcopter",
    enableMulticopterSimulation = True, enableVisionSimulation = True, remoteType = "RTM")
