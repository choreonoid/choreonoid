import WRSUtil
WRSUtil.loadProject(
    "SingleSceneView", "SP1", [ "AGXSimulator", "AISTSimulator" ], "Quadcopter",
    enableMulticopterSimulation = True, enableVisionSimulation = True, remoteType = "ROS")
