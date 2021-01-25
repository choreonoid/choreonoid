from cnoid.Util import *
from cnoid.Base import *
from cnoid.BodyPlugin import *
from cnoid.PythonPlugin import *
from numpy import *

rootItem = RootItem.instance

class BodyShaker:
    def __init__(self):
        self.connections = ScopedConnectionSet()

        toolBar = ToolBar("ShakeBar")
        self.button = toolBar.addToggleButton("Shake")
        self.button.setChecked(True)
        self.connections.add(self.button.toggled.connect(self.onButtonToggled))
        PythonPlugin.instance.mountToolBar(toolBar)

        self.bodyItems = []
        self.dp = array([0.0, 0.0, 0.01])
        self.connections.add(
            rootItem.sigSelectedItemsChanged.connect(self.onSelectionChanged))
        self.timer = Timer()
        self.connections.add(
            self.timer.timeout.connect(self.onTimeout))

    def onButtonToggled(self, on):
        self.onSelectionChanged(rootItem.selectedItems)

    def onSelectionChanged(self, items):
        self.bodyItems = BodyItemList(items)
        if len(self.bodyItems) > 0 and self.button.isChecked():
            self.timer.start(50)
        else:
            self.timer.stop()

    def onTimeout(self):
        for bodyItem in self.bodyItems:
            body = bodyItem.body
            body.rootLink.p += self.dp
            body.calcForwardKinematics()
            bodyItem.notifyKinematicStateChange()
        self.dp = -self.dp

try:
    # This is necessary to release the existing instance
    bodyShaker.connections.disconnect()
except NameError:
    pass

bodyShaker = BodyShaker()
