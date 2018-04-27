from cnoid.Base import *
from cnoid.BodyPlugin import *

simulatorItem = RootItem.getInstance().findItem("AISTSimulator")
ItemTreeView.getInstance().selectItem(simulatorItem)
simulatorItem.startSimulation()
