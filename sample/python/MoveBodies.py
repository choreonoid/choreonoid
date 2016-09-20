
from cnoid.Util import *
from cnoid.Base import *
from cnoid.BodyPlugin import *
from numpy import *
import time

bodyItems = RootItem.instance().getDescendantItems(BodyItem)

for bodyItem in bodyItems:
    body = bodyItem.body()
    rootLink = body.rootLink()
    for i in range(20):
        rootLink.p += array([0, 0, 0.01])
        body.calcForwardKinematics()
        bodyItem.notifyKinematicStateChange()
        MessageView.instance().flush()
        time.sleep(0.01)
