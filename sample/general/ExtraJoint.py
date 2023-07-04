from cnoid.Base import *
from cnoid.Body import *
from cnoid.BodyPlugin import *

sr1 = Item.find("SR1")
box = Item.find("box4")

ej = ExtraJoint(ExtraJoint.ExtraJointType.EJ_BALL)
ej.setLink(0, sr1.body.link("RARM_WRIST_R"))
ej.setLocalTranslation(0, [0.0, 0.0, -0.24])
ej.setLink(1, box.body.rootLink)
ej.setLocalTranslation(1, [0.0, 0.0, 0.1])

simulator = RootItem.instance.findItem(AISTSimulatorItem)
simulator.clearExtraJoints()
simulator.addExtraJoint(ej)
