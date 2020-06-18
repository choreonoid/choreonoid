from cnoid.Util import *
from cnoid.Base import *
from cnoid.BodyPlugin import *
import random
import time
import math

robotItem = None
for bodyItem in RootItem.instance.getDescendantItems(BodyItem):
    body = bodyItem.body
    nj = body.numJoints
    if nj >= 6:
        robotItem = bodyItem

if robotItem:
    print("\"{}\" has been detected as the target robot model "
          "for the kinematics test.".format(robotItem.name))
else:
    print("The target robot model was not found.")
    exit()

robot = robotItem.body
endLinks = []
for link in robot.links:
    if not link.child:
        endLinks.append(link)

random.seed(1)
for i in range(20):

    print("Joint angles: ", end="")
    for joint in robot.joints:
        joint.q = random.uniform(joint.q_lower, joint.q_upper)
        print("{:6.1f} ".format(math.degrees(joint.q)), end="")
    print()
    
    robot.calcForwardKinematics()
    
    for link in endLinks:
        print(" {}: ".format(link.name), end="")
        print(link.translation, end=" ")
        print(rpyFromRot(link.attitude))

    print(" Center of Mass: {}".format(robot.calcCenterOfMass()))

    robotItem.notifyKinematicStateChange()
    MessageView.instance.flush()
    time.sleep(0.2)

n = 10000000
print()
print("Measure the time it takes to do {} forward kinematics calculations".format(n))
random.seed(1)
start = time.time()
for i in range(10):
    for joint in robot.joints:
        joint.q = random.uniform(joint.q_lower, joint.q_upper)
    for j in range(n // 10):
        robot.calcForwardKinematics()
elapsed = time.time() - start
print("Total time: {:f} [s], average time: {:f} [us]".format(elapsed, (elapsed / n) * 1.0e6))
