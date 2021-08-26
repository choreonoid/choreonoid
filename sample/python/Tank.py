from cnoid.Util import *
from cnoid.Base import *
from cnoid.Body import *
from cnoid.BodyPlugin import *
import math

sceneWidget = SceneView.instance.sceneWidget
sceneWidget.setHeadLightEnabled(False)
sceneWidget.setFloorGridEnabled(False)
sceneWidget.setWorldLightIntensity(0.1)
sceneWidget.setWorldLightAmbient(0.0)
sceneWidget.setBackgroundColor([0, 0, 0])
sceneWidget.setCameraPosition(
    [ 6.85805, 3.70225, 2.89464 ],
    [ -0.834875, -0.460056, -0.302213 ],
    [ -0.264686, -0.145855, 0.95324 ])

joystickView = ViewManager.getOrCreateView("Base", "VirtualJoystickView")
MainWindow.instance.viewArea.addView(joystickView)
joystickView.bringToFront()

worldItem = WorldItem()
RootItem.instance.addChildItem(worldItem)

laboItem = BodyItem()
laboItem.load("${SHARE}/model/Labo1/Labo1v2.body")
laboItem.setChecked(True)
worldItem.addChildItem(laboItem)

lightingItem = LightingItem()
lightingItem.setLightType(LightingItem.SpotLight)
lightingItem.setTranslation([ 0.0, 0.0, 2.5 ])
lightingItem.setDirection([ 0.0, 0.0, -1.0 ])
lightingItem.setIntensity(0.8)
lightingItem.setBeamWidth(math.radians(60.0))
lightingItem.setCutOffAngle(math.radians(70.0))
lightingItem.setChecked(True)
worldItem.addChildItem(lightingItem)

tankItem = BodyItem()
tankItem.load("${SHARE}/model/Tank/Tank.body")
tankItem.setChecked(True)
tank = tankItem.body
tank.rootLink.setTranslation([ 2.0, 0.4, 0.106 ])
tank.rootLink.setRotation(rotFromRpy([0, 0, math.radians(180.0)]))
tank.calcForwardKinematics()
tankItem.storeInitialState()
worldItem.addChildItem(tankItem)

controllerItem = SimpleControllerItem()
controllerItem.setController("TankJoystickController")
tankItem.addChildItem(controllerItem)

simulatorItem = AISTSimulatorItem()
simulatorItem.setTimeStep(0.001)
simulatorItem.setRealtimeSyncMode(True)
simulatorItem.setTimeRangeMode(SimulatorItem.TR_UNLIMITED)
worldItem.addChildItem(simulatorItem)
simulatorItem.setSelected(True)

simulatorItem.startSimulation()
