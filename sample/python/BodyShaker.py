from cnoid.Util import *
from cnoid.Base import *
from cnoid.BodyPlugin import *
from cnoid.PythonPlugin import *
from cnoid.QtCore import *
from numpy import *

rootItem = RootItem.instance

class BodyShaker:
    def __init__(self):
        toolBar = ToolBar("ShakeBar")
        self.button = toolBar.addToggleButton("Shake")
        self.button.setChecked(True)
        self.buttonConnection = self.button.toggled.connect(self.onButtonToggled)
        PythonPlugin.instance.mountToolBar(toolBar)

        self.bodyItems = []
        self.dp = array([0.0, 0.0, 0.01])
        self.selectionConnection = rootItem.sigSelectedItemsChanged.connect(self.onSelectionChanged)
        self.timer = QTimer()
        self.timerConnection = self.timer.timeout.connect(self.onTimeout)

    def disconnect(self):
        # This is necessary to release the existing instance
        self.selectionConnection.disconnect()
        QObject.disconnect(self.buttonConnection)
        QObject.disconnect(self.timerConnection)

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
    bodyShaker.disconnect()
except NameError:
    pass

bodyShaker = BodyShaker()
