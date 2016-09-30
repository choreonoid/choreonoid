cnoid.require("Util")
cnoid.require("Base")

MyTask = { 
   new = function()
      local obj = { }
      obj.cppobj = cnoid.Task.new()
      setmetatable(obj, { __index = obj.cppobj })
      MyTask.initialize(obj)
      return obj
   end
}
setmetatable(MyTask, { __index = cnoid.Task })

function MyTask:initialize()
   
   self:setName("TestTask")

   self:addPhase("Phase 1")
   
   self:addCommand("Command 1")
       :setFunction(
          function(proc)
	     print "Command 1"
          end)
end

task = MyTask.new()

taskView = cnoid.TaskView.instance()
taskView:updateTask(task)
taskView:activate(true)
