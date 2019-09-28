import os
from cnoid.Util import *
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
    view, task, simulatorProjects, robotProjects,
    enableMulticopterSimulation = False, enableVisionSimulation = False, targetVisionSensors = "", remoteType = ""):

    projectdir = os.path.join(shareDirectory, "WRS2018", "project")
    #directory = os.path.dirname(os.path.realpath(__file__))
    
    itv = ItemTreeView.instance
    pm = ProjectManager.instance

    viewProject = SubProjectItem()
    viewProject.name = "ViewProject"
    viewProject.load(os.path.join(projectdir, view + ".cnoid"))
    RootItem.instance.addChildItem(viewProject)
    itv.expandItem(viewProject, False)

    world = WorldItem()
    world.name = "World"
    RootItem.instance.addChildItem(world)

    taskProject = SubProjectItem()
    taskProject.name = task
    taskProject.load(os.path.join(projectdir, task + ".cnoid"))
    world.addChildItem(taskProject)
    itv.expandItem(taskProject, False)

    if not isinstance(simulatorProjects, list):
        simulatorProjects = [ simulatorProjects ]
    for project in simulatorProjects:
        pm.loadProject(os.path.join(projectdir, project + ".cnoid"), world)

    # select only the first simulator item
    selectedSimulatorItems = SimulatorItemList(itv.getSelectedItems())
    for i in range(1, len(selectedSimulatorItems)):
        itv.selectItem(selectedSimulatorItems[i], False)

    if not isinstance(robotProjects, list):
        robotProjects = [ robotProjects ]

    robotOffset = 0.0

    for robotProject in robotProjects:

        robot = pm.loadProject(os.path.join(projectdir, robotProject + ".cnoid"), world)[0]

        rootLink = robot.body.rootLink;
        p = rootLink.translation
        p[1] -= robotOffset
        rootLink.setTranslation(p)
        robot.notifyKinematicStateChange(True)
        robot.storeInitialState()
        robotOffset += 1.5

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
            
    pm.setCurrentProjectName(task + "-" + robotProjects[0])
