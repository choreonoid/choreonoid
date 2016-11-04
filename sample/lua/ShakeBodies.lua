cnoid.require "Util"
cnoid.require "Base"
cnoid.require "Body"
cnoid.require "BodyPlugin"

self = { 
   connections = cnoid.ScopedConnectionSet.new()
}

local toolBar = cnoid.ToolBar.new("ShakeBar")
toolBar:setVisibleByDefault(true)
local button = toolBar:addToggleButton("Shake")
button:setChecked(true)
local bodyItems = { }
local timer = cnoid.Timer.new()
local dp = cnoid.Vector3(0.0, 0.0, 0.01)

local onSelectionChanged = function(items)
   bodyItems = cnoid.extractBodyItems(items)
   if #bodyItems > 0 and button:isChecked() then
      timer:start(50)
   else
      timer:stop()
   end
end
   
self.connections:add(
   cnoid.ItemTreeView.instance():sigSelectionChanged():connect(onSelectionChanged))

self.connections:add(
   button:sigToggled():connect(
      function(on)
	 onSelectionChanged(cnoid.ItemTreeView.instance():selectedItems())
      end))

self.connections:add(
   timer:sigTimeout():connect(
      function()
	 for i, bodyItem in ipairs(bodyItems) do
	    local body = bodyItem:body()
	    local rootLink = body:rootLink()
	    rootLink:setTranslation(rootLink:translation() + dp)
	    body:calcForwardKinematics()
	    bodyItem:notifyKinematicStateChange()
	    dp = -dp
	 end
      end))

self.finalize = function(self)
   self.connections:disconnect()
   cnoid.MainWindow.instance():removeToolBar(toolBar)
end

setmetatable(self, { __gc = self.finalize })

if bodyShaker then
   bodyShaker:finalize()
end

cnoid.MainWindow.instance():addToolBar(toolBar)
   
bodyShaker = self
