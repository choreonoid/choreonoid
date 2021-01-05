from cnoid.Base import *
from cnoid.Body import *
from cnoid.BodyPlugin import *

robotItem = None
for bodyItem in RootItem.instance.getDescendantItems(BodyItem):
    legged = LeggedBodyHelper(bodyItem.body)
    if legged.numFeet == 2:
        robotItem = bodyItem
        break

newRobotItem = robotItem.duplicate()
robot = newRobotItem.body

for link in robot.links:
    if not link.isBodyRoot():
        if link.isRevoluteJoint():
            link.setJointType(Link.PrismaticJoint)
        elif link.isPrismaticJoint():
            link.setJointType(Link.FixedJoint)
        elif link.isFixedJoint():
            link.setJointType(Link.RevoluteJoint)

newRobotItem.replace(robotItem)
