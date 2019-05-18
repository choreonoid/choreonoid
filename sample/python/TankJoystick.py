from cnoid.Util import *
from cnoid.Base import *
from cnoid.Body import *
from cnoid.BodyPlugin import *
import math;

sceneWidget = SceneView.instance.sceneWidget
sceneWidget.setHeadLightEnabled(False)
sceneWidget.setFloorGrid(False)
sceneWidget.setWorldLightIntensity(0.1)
sceneWidget.setWorldLightAmbient(0.0)
sceneWidget.setBackgroundColor([0, 0, 0])
sceneWidget.setCameraPosition(
    [ -2.86824,   6.25331,   2.49127  ],
    [  0.412288, -0.847325, -0.334751 ],
    [  0.146464, -0.301009,  0.942307 ])

joystickView = ViewManager.getOrCreateView("Base", "VirtualJoystickView")
MainWindow.instance.viewArea.addView(joystickView)
joystickView.bringToFront()

itv = ItemTreeView.instance

worldItem = WorldItem()
RootItem.instance.addChildItem(worldItem)

laboItem = BodyItem()
laboItem.load(shareDirectory + "/model/Labo1/Labo1.body")
worldItem.addChildItem(laboItem)
itv.checkItem(laboItem)

tankItem = BodyItem()
tankItem.load(shareDirectory + "/model/Tank/Tank.body")
tank = tankItem.body
tank.rootLink.setTranslation([-0.8, 2.4, 0.1])
tank.rootLink.setRotation(rotFromRpy([0, 0, math.radians(-90.0)]))
tank.calcForwardKinematics()
tankItem.storeInitialState()
worldItem.addChildItem(tankItem)
itv.checkItem(tankItem)

controllerItem = SimpleControllerItem()
controllerItem.setController("TankJoystickController")
tankItem.addChildItem(controllerItem)

simulatorItem = AISTSimulatorItem()
simulatorItem.setTimeStep(0.001)
simulatorItem.setRealtimeSyncMode(True)
simulatorItem.setTimeRangeMode(SimulatorItem.TimeRangeMode.TR_UNLIMITED)
worldItem.addChildItem(simulatorItem)
itv.selectItem(simulatorItem)

simulatorItem.startSimulation()
