cnoid.require("Util")
cnoid.require("Base")

TaskBase = cnoid.derive(cnoid.Task)

function TaskBase:initialize()
   print "TaskBase:initialize()"
   self:setName("TaskBase")
end

function TaskBase:onActivated(sequencer)
   self.super:onActivated(sequencer)
   print "TaskBase::onActivated()"
end

function TaskBase:onDeactivated(sequencer)
   print "TaskBase::onDeactivated()"
   self.super:onDeactivated(sequencer)
end

function TaskBase:storeState(sequencer, archive)
   self.super:storeState(sequencer, archive)
   print "TaskBase::storeState()"
   print("currentPhaseIndex: ", sequencer:currentPhaseIndex())
   print("archive:nodeType(): ", archive:nodeType())
   print("archive:size(): ", archive:size())
   print("archive:empty(): ", archive:empty())
   print("archive:numberFormat(): ", archive:numberFormat())
   print("archive:isMapping(): ", archive:isMapping())
   print("archive:isListing(): ", archive:isListing())
end

function TaskBase:restoreState(sequencer, archive)
   self.super:restoreState(sequencer, archive)
   print "TaskBase::restoreState()"
   print("archive:nodeType(): ", archive:nodeType())
end



TestTask = cnoid.derive(TaskBase)

function TestTask:initialize()
  
  print "TestTask::initialize()"
   
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
  print "TestTask:onActivated()"
end

function TestTask:onDeactivated(sequencer)
  print "TestTask:onDeactivated()"
  self.super:onDeactivated(sequencer)
end

task = TestTask.new()

task2 = TestTask.new()
task2:setName("Task 2");

taskView = cnoid.TaskView.instance()
taskView:updateTask(task)
taskView:updateTask(task2)
taskView:activate(true)
