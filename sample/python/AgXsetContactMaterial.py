from cnoid.Base import *
from cnoid.BodyPlugin import *
from cnoid.AgXPlugin import *

sr1 = Item.find("SR1").body()
box = Item.find("box2").body()
simulator = Item.find("AgXSimulator-Velocity")

simulator.setContactMaterialParam(sr1.link("LARM_WRIST_R"), box.link("WAIST"), 0.001, 2e10 )
simulator.setContactMaterialParam(sr1.link("RARM_WRIST_R"), box.link("WAIST"), 0.001, 2e10 )
