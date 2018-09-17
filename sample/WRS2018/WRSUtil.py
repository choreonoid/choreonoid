import os
from cnoid.Base import *
from cnoid.BodyPlugin import *

try:
    from cnoid.MulticopterPlugin import *
except:
    pass

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
    viewProject, worldProject, simulatorProjects, robotProject,
    enableMulticopterSimulation = False, enableVisionSimulation = False, targetVisionSensors = "", remoteType = ""):

    directory = os.path.dirname(os.path.realpath(__file__))
    
    pm = ProjectManager.instance

    pm.loadProject(os.path.join(directory, viewProject + ".cnoid"))

    pm.loadProject(os.path.join(directory, worldProject + ".cnoid"))

    world = Item.find("World")

    if not isinstance(simulatorProjects, list):
        simulatorProjects = [ simulatorProjects ]
    for project in simulatorProjects:
        pm.loadProject(os.path.join(directory, project + ".cnoid"), world)

    # select only the first simulator item
    itv = ItemTreeView.instance
    selectedSimulatorItems = SimulatorItemList(itv.getSelectedItems())
    for i in range(1, len(selectedSimulatorItems)):
        itv.selectItem(selectedSimulatorItems[i], False)

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

    if enableMulticopterSimulation:
        multicopterSimulator = MulticopterSimulatorItem()
        simulators = world.getDescendantItems(SimulatorItem)
        for simulator in simulators:
            simulator.addChildItem(multicopterSimulator.duplicate())

    if enableVisionSimulation:
        visionSimulator = GLVisionSimulatorItem()
        visionSimulator.setTargetSensors(targetVisionSensors)
        visionSimulator.setBestEffortMode(True)
        simulators = world.getDescendantItems(SimulatorItem)
        for simulator in simulators:
            simulator.addChildItem(visionSimulator.duplicate())
            
    pm.setCurrentProjectName(worldProject + "-" + robotProject)
