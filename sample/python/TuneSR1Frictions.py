from cnoid.Base import *
from cnoid.BodyPlugin import *

sr1 = Item.find("SR1").body()
floorLink = Item.find("Floor").body().rootLink()
simulator = Item.find("AISTSimulator")

simulator.setFriction(sr1.link("LLEG_ANKLE_R"), floorLink, 1.0, 1.0)
simulator.setFriction(sr1.link("RLEG_ANKLE_R"), floorLink, 0.0, 0.0)
