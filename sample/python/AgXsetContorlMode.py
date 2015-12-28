from cnoid.Base import *
from cnoid.BodyPlugin import *
from cnoid.AgXPlugin import *

sr1 = Item.find("SR1").body()
simulator = Item.find("AgXSimulator")

simulator.setJointControlMode(sr1.link("LARM_ELBOW"), simulator.ControlMode.TORQUE)
simulator.setJointControlMode(sr1.link("RARM_ELBOW"), simulator.ControlMode.TORQUE)
