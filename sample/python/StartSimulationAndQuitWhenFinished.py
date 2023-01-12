from cnoid.Base import *
from cnoid.BodyPlugin import *

def onSimulationFinished(isForced):
    App.exit()

simulatorItem = RootItem.instance.findItem(SimulatorItem)
if not simulatorItem:
    App.exit()

simulatorItem.sigSimulationFinished.connect(onSimulationFinished)
simulatorItem.setRealtimeSyncMode(SimulatorItem.NonRealtimeSync)
simulatorItem.setSelected()
simulatorItem.startSimulation()
