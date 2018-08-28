import WRSUtil
WRSUtil.loadProject(
    "SingleSceneView", "SP", "AISTSimulator", "DoubleArmV7S",
    enableVisionSimulation = True, targetVisionSensors = "FRAME_FRONT_CAMERA", remoteType = "ROS")
