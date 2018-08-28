import WRSUtil
WRSUtil.loadProject(
    "SingleSceneView", "T2", "AISTSimulator", "DoubleArmV7S",
    enableVisionSimulation = True, targetVisionSensors = "FRAME_FRONT_CAMERA", remoteType = "ROS")
