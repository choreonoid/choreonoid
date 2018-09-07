from cnoid.Util import *
import cnoid.Base

def doCommand3_1(proc):
    print("Command 3-1")

def doCommand3_2():
    print("Command 3-2")
    

class TestTask(Task):

    def __init__(self):
        Task.__init__(self)

        self.setName("TestTask")
        self.mv = cnoid.Base.MessageView.getInstance()

        self.setupSequence()

    def doCommand2_1(self):
        print("Command 2-1")

    def doCommand2_2(self, proc):
        print("Command 2-2")
        
    def onActivated(self, sequencer):
        self.mv.putln("TestTask.onActivated")

    def onDeactivated(self, sequencer):
        self.mv.putln("TestTask.onDeactivated")

    def storeState(self, sequencer, archive):
        self.mv.putln("TestTask.storeState")
        
    def restoreState(self, sequencer, archive):
        self.mv.putln("TestTask.restoreState")

    def setupSequence(self):

        self.addPhase("Phase 1")

        self.addCommand("Command 1-1")\
            .setDefault()\
            .setFunction(lambda proc : self.mv.putln("Command 1-1"))\
            .linkToNextCommand()
        
        self.addCommand("Command 1-2")\
            .setFunction(lambda : self.mv.putln("Command 1-2"))\
            .linkToNextPhase()

        self.addPhase("Phase 2")
        
        self.addCommand("Command 2-1")\
            .setDefault()\
            .setFunction(self.doCommand2_1)\
            .linkToNextCommand()
        
        self.addCommand("Command 2-2")\
            .setFunction(self.doCommand2_2)\
            .linkToNextPhase()

        self.addPhase("Phase 3")
        
        self.addCommand("Command 3-1")\
            .setDefault()\
            .setFunction(doCommand3_1)\
            .linkToNextCommand()
        
        self.addCommand("Command 3-2")\
            .setFunction(doCommand3_2)\
            .linkToNextTask()
        
class TestTask2(Task):

    def __init__(self):
        Task.__init__(self)

        self.setName("TestTask2")
        self.mv = cnoid.Base.MessageView.getInstance()

        self.setupSequence()

    def onActivated(self, sequencer):
        self.mv.putln("TestTask2.onActivated")

    def onDeactivated(self, sequencer):
        self.mv.putln("TestTask2.onDeactivated")

    def storeState(self, sequencer, archive):
        self.mv.putln("TestTask2.storeState")
        
    def restoreState(self, sequencer, archive):
        self.mv.putln("TestTask2.restoreState")

    def setupSequence(self):

        self.addPhase("Phase 1")
        
        self.addCommand("Command 1")\
            .setDefault()\
            .setFunction(lambda proc : self.mv.putln("Command 1"))\
            .linkToNextPhase()

        self.addPhase("Phase 2")
        
        self.addCommand("Command 2")\
            .setDefault()\
            .setFunction(lambda proc : self.mv.putln("Command 2"))\
            .linkToNextCommand()
        
        self.addCommand("Command 3")\
            .setFunction(lambda proc : self.mv.putln("Command 3"))\
            .linkToNextPhase()

        self.addPhase("Phase 2")

        self.addCommand("Finish")
            
        
        
taskView = cnoid.Base.TaskView.getInstance()
taskView.updateTask(TestTask())
taskView.updateTask(TestTask2())

taskView.serializeTasks(["TestTask", "TestTask2"])

taskView.activate(True)
