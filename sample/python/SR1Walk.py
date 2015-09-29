
from cnoid.Util import *
from cnoid.Base import *
from cnoid.Body import *
from cnoid.BodyPlugin import *
from cnoid.SimpleControllerPlugin import *
import math;

worldItem = WorldItem()
RootItem.instance().addChildItem(worldItem)

timeBar = TimeBar.instance()
timeBar.setFrameRate(500)
timeBar.setTimeRange(0.0, 15.0)
timeBar.setFillLevelSync(False)

robotItem = BodyItem()
robotItem.load(shareDirectory() + "/model/SR1/SR1.yaml")

robot = robotItem.body()
robot.rootLink().setTranslation( [0.0, 0.0, 0.7135] )

q = [ 0.0, -2.1, 0.0,   4.5, -2.4, 0.0,
     10.0, -0.2, 0.0, -90.0,  0.0, 0.0, 0.0,
      0.0, -2.1, 0.0,   4.5, -2.4, 0.0,
     10.0, -0.2, 0.0, -90.0,  0.0, 0.0, 0.0,
      0.0,  0.0, 0.0 ]

for i in range(robot.numJoints()):
	robot.joint(i).q = math.radians(q[i])

robot.calcForwardKinematics()
robotItem.storeInitialState()

controllerItem = SimpleControllerItem()
controllerItem.setControllerDllName("SR1WalkPatternController")
robotItem.addChildItem(controllerItem)

worldItem.addChildItem(robotItem)

ItemTreeView.instance().checkItem(robotItem)

floorItem = BodyItem()
floorItem.load(shareDirectory() + "/model/misc/floor.wrl")
worldItem.addChildItem(floorItem)

simulatorItem = AISTSimulatorItem()
worldItem.addChildItem(simulatorItem)
ItemTreeView.instance().selectItem(simulatorItem)

simulatorItem.startSimulation()
