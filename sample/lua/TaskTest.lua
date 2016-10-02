cnoid.require("Util")
cnoid.require("Base")

function deriveCppClass(base)
  local class = { }
  class.new = function(...)
      local obj = { }
      obj.cppobj = base.new(...)
      setmetatable(obj, { __index = class })
      obj.cppobj:setDescendantLuaObject(obj)
      MyTask.initialize(obj)
      return obj
  end
  setmetatable(class, { __index = base })
  return class
end

MyTask = deriveCppClass(cnoid.Task)

function MyTask:initialize()
   
   self:setName("TestTask")

   self:addPhase("Phase 1")
   
   self:addCommand("Command 1")
       :setFunction(
          function(proc)
	     print "Command 1"
          end)
end

function MyTask:onActivated()
  print "MyTask:onActivated()"
end

function MyTask:onDeactivated()
  print "MyTask:onDeactivated()"
end

task = MyTask.new()

task2 = MyTask.new()
task2:setName("Task 2");

taskView = cnoid.TaskView.instance()
taskView:updateTask(task)
taskView:updateTask(task2)
taskView:activate(true)
