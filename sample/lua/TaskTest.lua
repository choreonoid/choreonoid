cnoid.require "Util"
cnoid.require "Base"

TaskBase = cnoid.derive(cnoid.Task)

TestTask = cnoid.derive(TaskBase)

function TestTask:initialize()
  
  self:setName("TestTask")

  self:addPhase("Phase 1")

  self:addCommand("Command 1")
       :setFunction(
          function(proc)
	     print "Command 1"
          end)
end

function TestTask:onActivated(sequencer)
  self.super:onActivated(sequencer)
end

function TestTask:onDeactivated(sequencer)
  self.super:onDeactivated(sequencer)
end

function TestTask:storeState(sequencer, archive)
   self.super:storeState(sequencer, archive)
end

function TestTask:restoreState(sequencer, archive)
   self.super:restoreState(sequencer, archive)
end

task = TestTask.new()

task2 = TestTask.new()
task2:setName("Task 2");

taskView = cnoid.TaskView.instance()
taskView:updateTask(task)
taskView:updateTask(task2)
taskView:activate(true)
