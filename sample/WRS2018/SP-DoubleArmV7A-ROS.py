import WRSUtil
WRSUtil.loadProject(
    "SingleSceneView", "SP", "AGXSimulator", "DoubleArmV7A",
    enableVisionSimulation = True, targetVisionSensors = "FRAME_FRONT_CAMERA", remoteType = "ROS")
