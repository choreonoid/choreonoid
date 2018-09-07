from cnoid.Base import *
from cnoid.Body import *
from cnoid.BodyPlugin import *

ej = ExtraJoint(ExtraJoint.ExtraJointType.EJ_BALL, [0,0,0])
ej.setPoint(0, "SR1", "RARM_WRIST_R", [0.0, 0.0, -0.24])
ej.setPoint(1, "box4", "WAIST",  [0.0, 0.0, 0.1])

simulator = RootItem.instance().find("World/AISTSimulator")
simulator.clearExtraJoint()
simulator.addExtraJoint(ej)
