
from cnoid.Base import *
from cnoid.BodyPlugin import *

robotItem = RootItem.instance().findChildItem("World/SR1")
simulatorItem = SimulatorItem.findActiveSimulatorItemFor(robotItem)
waistLink = robotItem.body().link("WAIST")
simulatorItem.setExternalForce(robotItem, waistLink, [0, 0, 0], [200, 0.0, 0.0])
