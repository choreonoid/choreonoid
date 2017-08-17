from cnoid.Util import *
import cnoid.Base

class TestTask(Task):

    def __init__(self):
        Task.__init__(self)

        self.setName("TestTask")
        self.mv = cnoid.Base.MessageView.instance()

        self.setupSequence()

    def onActivated(self, sequencer):
        self.mv.putln("TestTask.onActivated")

    def onDeactivated(self, sequencer):
        self.mv.putln("TestTask.onDeactivated")

    def storeState(self, sequencer, archive):
        self.mv.putln("TestTask.storeState")
        #self.mv.putln("sequencer: {0}".format(sequencer))
        #global seq
        #seq = sequencer
        self.mv.putln("Phase index is {0}".format(sequencer.currentPhaseIndex()))
        
    def restoreState(self, sequencer, archive):
        self.mv.putln("TestTask.restoreState")

    def setupSequence(self):

        self.addPhase("Phase 1")
        self.addCommand("Command 1").setFunction(lambda proc : self.mv.putln("Command 1"))

        self.addPhase("Phase 2")
        self.addCommand("Command 2").setFunction(lambda proc : self.mv.putln("Command 2"))
        self.addCommand("Command 3").setFunction(lambda proc : self.mv.putln("Command 3"))

taskView = cnoid.Base.TaskView.instance()
taskView.updateTask(TestTask())
taskView.activate(True)
