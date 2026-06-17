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

# Register the extra joint to each body. When a simulator clones bodies through
# a shared CloneMap (as AISTSimulatorItem does), the link references inside the
# extra joint are properly translated to those of the cloned bodies.
sr1.body.clearExtraJoints()
sr1.body.addExtraJoint(ej)
box.body.clearExtraJoints()
box.body.addExtraJoint(ej)
