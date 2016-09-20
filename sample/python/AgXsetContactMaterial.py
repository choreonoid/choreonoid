from cnoid.Base import *
from cnoid.BodyPlugin import *
from cnoid.AgXPlugin import *

sr1 = Item.find("SR1").body()
box = Item.find("box2").body()
simulator = Item.find("AgXSimulator-Velocity")

simulator.setContactMaterialDamping(sr1, box, 0.001)
simulator.setContactMaterialYoungsModulus(sr1, box, 2e10 )
