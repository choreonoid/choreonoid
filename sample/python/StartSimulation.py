from cnoid.Base import *
from cnoid.BodyPlugin import *

simulatorItem = Item.find("AISTSimulator")
ItemTreeView.instance.selectItem(simulatorItem)
simulatorItem.startSimulation()
