import WRSUtil
WRSUtil.loadProject(
    "SingleSceneView", "T1", [ "AGXSimulator", "AISTSimulator" ], "Quadcopter",
    enableMulticopterSimulation = True, enableVisionSimulation = True, remoteType = "RTM")
