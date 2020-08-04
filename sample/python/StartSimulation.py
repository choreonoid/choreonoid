from cnoid.Base import *
from cnoid.BodyPlugin import *

simulatorItem = Item.find("AISTSimulator")
RootItem.instance.selectItem(simulatorItem)
simulatorItem.startSimulation()
