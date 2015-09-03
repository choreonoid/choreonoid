from cnoid.Base import *
from cnoid.BodyPlugin import *

simulatorItem = RootItem.instance().findItem("AISTSimulator")
ItemTreeView.instance().selectItem(simulatorItem)
simulatorItem.startSimulation()
