import os
from cnoid.Base import *
from cnoid.BodyPlugin import *

try:
    from cnoid.OpenRTMPlugin import *
except:
    pass
try:
    from cnoid.AGXDynamicsPlugin import *
except:
    pass
try:
    from cnoid.ROSPlugin import *
except:
    pass

def loadProject(
    worldProject, simulatorProject, robotProject,
    enableVisionSimulator = False, targetVisionSensors = "", remoteType = ""):

    directory = os.path.dirname(os.path.realpath(__file__))
    
    pm = ProjectManager.instance

    pm.loadProject(os.path.join(directory, worldProject + ".cnoid"))

    world = Item.find("World")

    pm.loadProject(os.path.join(directory, simulatorProject + ".cnoid"), world)

    robot = pm.loadProject(os.path.join(directory, robotProject + ".cnoid"), world)[0]

    if remoteType:
        joystickInput = SimpleControllerItem()
        joystickInput.name = robot.name + "-JoystickInput"
        mainController = robot.getDescendantItems(SimpleControllerItem)[0]
        mainController.addChildItem(joystickInput)

        if remoteType == "RTM":
            joystickInput.setController("RemoteJoystickInputController")
            visionSensorOutput = BodyIoRTCItem()
            visionSensorOutput.name = "VisionSensorOutput"
            visionSensorOutput.rtcModuleName = "VisionSensorIoRTC"
            robot.addChildItem(visionSensorOutput)
        
        elif remoteType == "ROS":
            joystickInput.setController("JoyTopicSubscriberController")
            bodyPublisher = BodyPublisherItem()
            bodyPublisher.name = "BodyPublisher"
            robot.addChildItem(bodyPublisher)

    if enableVisionSimulator:
        visionSimulator = GLVisionSimulatorItem()
        visionSimulator.setTargetSensors(targetVisionSensors)
        simulators = world.getDescendantItems(SimulatorItem)
        for simulator in simulators:
            simulator.addChildItem(visionSimulator.duplicate())
            
    pm.setCurrentProjectName(worldProject + "-" + robotProject)
