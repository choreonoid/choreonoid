from cnoid.Util import *
from cnoid.Base import *
from cnoid.Body import *
from cnoid.BodyPlugin import *
import math;

worldItem = WorldItem()
RootItem.instance.addChildItem(worldItem)

robotItem = BodyItem()
robotItem.load(shareDirectory + "/model/SR1/SR1.body")

robot = robotItem.body
robot.rootLink.setTranslation([0.0, 0.0, 0.7135])

q = [ 0.0, -2.1, 0.0,   4.5, -2.4, 0.0,
     10.0, -0.2, 0.0, -90.0,  0.0, 0.0, 0.0,
      0.0, -2.1, 0.0,   4.5, -2.4, 0.0,
     10.0, -0.2, 0.0, -90.0,  0.0, 0.0, 0.0,
      0.0,  0.0, 0.0 ]

for i in range(robot.numJoints):
    robot.joint(i).q = math.radians(q[i])

robot.calcForwardKinematics()
robotItem.storeInitialState()

controllerItem = SimpleControllerItem()
controllerItem.setController("SR1WalkPatternController")
robotItem.addChildItem(controllerItem)

worldItem.addChildItem(robotItem)

ItemTreeView.instance.checkItem(robotItem)

floorItem = BodyItem()
floorItem.load(shareDirectory + "/model/misc/floor.body")
worldItem.addChildItem(floorItem)

simulatorItem = AISTSimulatorItem()
simulatorItem.setTimeStep(0.002)
simulatorItem.setTimeRangeMode(SimulatorItem.TimeRangeMode.ACTIVE_CONTROL)
worldItem.addChildItem(simulatorItem)
ItemTreeView.instance.selectItem(simulatorItem)

simulatorItem.startSimulation()
