
from cnoid.Util import *
from cnoid.Base import *
from cnoid.Body import *
from cnoid.BodyPlugin import *
from cnoid.SimpleControllerPlugin import *
import math;

worldItem = WorldItem()
RootItem.instance().addChildItem(worldItem)

timeBar = TimeBar.instance()
timeBar.setFrameRate(1000)
timeBar.setFillLevelSync(False)

sceneWidget = SceneView.instance().sceneWidget()
sceneWidget.setHeadLightEnabled(False)
sceneWidget.setFloorGrid(False)
sceneWidget.setWorldLightIntensity(0.1)
sceneWidget.setWorldLightAmbient(0.0)
sceneWidget.setBackgroundColor([0, 0, 0])
sceneWidget.setCameraPosition(
    [ -2.86824,   6.25331,   2.49127  ],
    [  0.412288, -0.847325, -0.334751 ],
    [  0.146464, -0.301009,  0.942307 ])

laboItem = BodyItem()
laboItem.load(shareDirectory() + "/model/Labo1/Labo1.wrl")
worldItem.addChildItem(laboItem)
ItemTreeView.instance().checkItem(laboItem)

tankItem = BodyItem()
tankItem.load(shareDirectory() + "/model/misc/tank.wrl")
tank = tankItem.body()
tank.rootLink().setTranslation([-0.8, 2.4, 0.1])
tank.rootLink().setRotation(rotFromRpy([0, 0, math.radians(-90.0)]))
tank.calcForwardKinematics()
tankItem.storeInitialState()
worldItem.addChildItem(tankItem)
ItemTreeView.instance().checkItem(tankItem)

controllerItem = SimpleControllerItem()
controllerItem.setControllerDllName("TankJoystickController")
tankItem.addChildItem(controllerItem)

simulatorItem = AISTSimulatorItem()
simulatorItem.setRealtimeSyncMode(True)
simulatorItem.setTimeRangeMode(SimulatorItem.TimeRangeMode.UNLIMITED)
worldItem.addChildItem(simulatorItem)
ItemTreeView.instance().selectItem(simulatorItem)

simulatorItem.startSimulation()
