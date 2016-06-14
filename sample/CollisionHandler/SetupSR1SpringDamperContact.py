from cnoid.Base import *
from cnoid.BodyPlugin import *

sr1 = Item.find("SR1").body()
floorLink = Item.find("Floor").body().rootLink()
simulator = Item.find("AISTSimulator")
handler = simulator.collisionHandlerId("SpringDamperContact")

simulator.setCollisionHandler(sr1.link("LLEG_ANKLE_R"), floorLink, handler)
simulator.setCollisionHandler(sr1.link("RLEG_ANKLE_R"), floorLink, handler)

