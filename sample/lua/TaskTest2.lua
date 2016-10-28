cnoid.require("Util")
cnoid.require("Base")

function deriveCppClass(baseClass)
   local class = { }
   class.new = function(...)
      local obj = { }
      baseobj = baseClass.new(...)
      if type(baseobj) == "table" then
	obj.cppobj = baseobj.cppobj
      else
	obj.cppobj = baseobj
      end
      obj.super = baseobj;
      setmetatable(obj, { __index = class })
      obj.cppobj:setDerivedLuaObject(obj)
      class.initialize(obj)
      return obj
   end
   setmetatable(class, { __index = baseClass })
   return class
end

TaskBase = deriveCppClass(cnoid.Task)

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


TestTask = deriveCppClass(TaskBase)


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
