import WRSUtil
WRSUtil.loadProject(
    "SP1", "AGXSimulator", "AizuSpiderSA", 
    enableVisionSimulation = True, targetVisionSensors = "FRONT_CAMERA, ARM_CAMERA", remoteType = "ROS")
