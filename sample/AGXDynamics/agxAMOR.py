# run choreonoid --python agxAMOR.py

from cnoid.Util import *
from cnoid.Base import *
from cnoid.Body import *
from cnoid.BodyPlugin import *
from cnoid.AGXDynamicsPlugin import *
from cnoid.ODEPlugin import *
import random
import math

pi = math.pi

sceneWidget = SceneView.instance.sceneWidget()
sceneWidget.setCameraPosition(
    [ 18.761382, 6.07919682, 8.55293262 ],
    [ -0.943178266, -0.285207267, -0.170503879 ],
    [ -0.163205368, -0.0493516004, 0.985357005 ])


worldItem = WorldItem()
RootItem.instance.addChildItem(worldItem)

floorItem = BodyItem()
floorItem.load(shareDirectory + "/model/misc/floor.body")
worldItem.addChildItem(floorItem)
ItemTreeView.instance.checkItem(floorItem)


random.seed( 32 )
def random_translation():
	return [random.uniform( -2, 2 ),
        random.uniform( -2, 2 ),
        random.uniform(  0, 10 ) ]

def random_rotation():
    return [random.uniform( -0.5 * pi, 0.5 * pi ),
        random.uniform( -0.5 * pi, 0.5 * pi ),
        random.uniform( -0.5 * pi, 0.5 * pi ),  ]


num_bodies = 200
for i in range(0, num_bodies):
    rodItem = BodyItem()
    rodItem.load(shareDirectory + "/model/misc/rod.body")
    rodItem.setName(rodItem.name()+str(i))
    rod = rodItem.body
    rod.rootLink.setTranslation(random_translation())
    rod.rootLink.setRotation(rotFromRpy(random_rotation()))
    rodItem.storeInitialState()
    worldItem.addChildItem(rodItem)
    ItemTreeView.instance.checkItem(rodItem)


simulatorItem = AGXSimulatorItem()
simulatorItem.setTimeStep(0.005)
simulatorItem.setEnableAMOR(True)
simulatorItem.setRealtimeSyncMode(True)
simulatorItem.setTimeRangeMode(SimulatorItem.TimeRangeMode.TR_UNLIMITED)
worldItem.addChildItem(simulatorItem)
ItemTreeView.instance.selectItem(simulatorItem)

#simulatorItem.startSimulation()
