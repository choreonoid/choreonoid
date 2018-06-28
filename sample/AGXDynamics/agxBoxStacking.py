# run choreonoid --python agxAMOR.py

from cnoid.Util import *
from cnoid.Base import *
from cnoid.Body import *
from cnoid.BodyPlugin import *
from cnoid.AGXDynamicsPlugin import *
from cnoid.ODEPlugin import *
import math

pi = math.pi

sceneWidget = SceneView.instance.sceneWidget()
sceneWidget.setCameraPosition(
    [ 3.80973835, -7.39332055, 7.71594622 ],
    [ -0.00358733323, 0.807228766, -0.590227795 ],
    [ -0.0026229527, 0.590221967, 0.807236737 ])

worldItem = WorldItem()
RootItem.instance.addChildItem(worldItem)

floorItem = BodyItem()
floorItem.load(shareDirectory + "/model/misc/floor.body")
floorBody = floorItem.body
floorBody.rootLink.setTranslation([4, 4, 0.0])
floorItem.storeInitialState()
worldItem.addChildItem(floorItem)
ItemTreeView.instance.checkItem(floorItem)

width = 10
height = 10
offset = 0.2
px = 0.0
py = 0.0
pz = 0.5
id = 0

def generate_id():
    global id
    id += 1
    return id

def create_body():
    bodyItem = BodyItem()
    bodyItem.load(shareDirectory + "/model/misc/box4.body")
    bodyItem.setName(bodyItem.name+"_"+str(generate_id()))
    worldItem.addChildItem(bodyItem)
    ItemTreeView.instance.checkItem(bodyItem)
    return bodyItem

def create_bodies(length):
    global px, py
    for i in range(0, length):
        for j in range(0, length):
            bodyItem = create_body()
            bodyItem.body.rootLink.setTranslation([px, py, pz])
            bodyItem.storeInitialState()
            bodyItem.restoreInitialState(True)
            py += offset
        px += offset
        py = 0.0
    px = 0.0

for k in range(0, height):
    create_bodies(width)
    pz += offset

simulatorItem = AGXSimulatorItem()
simulatorItem.setTimeStep(0.005)
simulatorItem.setNumThreads(4)
simulatorItem.setEnableContactWarmstarting(True)
simulatorItem.setEnableAMOR(True)
simulatorItem.setRealtimeSyncMode(True)
#simulatorItem.setTimeRangeMode(SimulatorItem.TimeRangeMode.TR_UNLIMITED)
simulatorItem.setTimeRangeMode(simulatorItem.TimeRangeMode.SPECIFIED)
simulatorItem.setSpecifiedRecordingTimeLength(5.0)
worldItem.addChildItem(simulatorItem)
ItemTreeView.instance.selectItem(simulatorItem)

simulatorItem.startSimulation()
