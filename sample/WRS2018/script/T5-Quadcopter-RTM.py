import WRSUtil
WRSUtil.loadProject(
    "SingleSceneView", "T5", [ "AGXSimulator", "AISTSimulator" ], "Quadcopter",
    enableMulticopterSimulation = True, enableVisionSimulation = True, remoteType = "RTM")
