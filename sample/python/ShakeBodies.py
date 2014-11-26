
from cnoid.Util import *
from cnoid.Base import *
from cnoid.BodyPlugin import *
from numpy import *

class ShakeBodies:
    def __init__(self):
        self.bodyItems = []
        self.dp = array([0.0, 0.0, 0.01])
        self.connections = ScopedConnectionSet()
        self.connections.add(
            ItemTreeView.instance().sigSelectionChanged().connect(self.onSelectionChanged))
        self.timer = Timer()
        self.connections.add(
            self.timer.sigTimeout().connect(self.onTimeout))

    def onSelectionChanged(self, items):
        self.bodyItems = BodyItemList(items)
        if len(self.bodyItems) > 0:
            self.timer.start(50)
        else:
            self.timer.stop()

    def onTimeout(self):
        for bodyItem in self.bodyItems:
            bodyItem.body().rootLink().p += self.dp
            bodyItem.notifyKinematicStateChange(True)
        self.dp = -self.dp

shakeBodies = ShakeBodies()
