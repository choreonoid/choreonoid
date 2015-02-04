
from cnoid.Util import *
from cnoid.Base import *
from cnoid.BodyPlugin import *
from numpy import *
import time

bodyItems = RootItem.instance().getDescendantItems(BodyItem)
dp = array([0, 0, 0.01])

for bodyItem in bodyItems:
    for i in range(20):
        body = bodyItem.body
        root = body.rootLink
        root.p += dp
        bodyItem.notifyKinematicStateChange(True)
        MessageView.instance().flush()
        time.sleep(0.01)
