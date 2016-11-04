cnoid.require "Util"
cnoid.require "Base"
cnoid.require "Body"
cnoid.require "BodyPlugin"

BodyShaker = {
   new = function()
      local self = { }
      self.toolBar = cnoid.ToolBar.new("ShakeBar")
      self.toolBar:setVisibleByDefault(true)
      self.button = self.toolBar:addToggleButton("Shake")
      self.button:setChecked(true)

      self.bodyItems = { }
      self.dp = cnoid.Vector3(0.0, 0.0, 0.01)
      self.connections = cnoid.ScopedConnectionSet.new()
      self.timer = cnoid.Timer.new()

      function onSelectionChanged(items)
	 self.bodyItems = cnoid.extractBodyItems(items)
	 if #self.bodyItems > 0 and self.button:isChecked() then
	    self.timer:start(50)
	 else
	    self.timer:stop()
	 end
      end

      self.connections:add(
	 cnoid.ItemTreeView.instance():sigSelectionChanged():connect(onSelectionChanged))

      self.connections:add(
	 self.button:sigToggled():connect(
	    function(on)
	       onSelectionChanged(cnoid.ItemTreeView.instance():selectedItems())
	    end))

      self.onTimeout = function()
	 for i, bodyItem in ipairs(self.bodyItems) do
	    local body = bodyItem:body()
	    body:rootLink():setTranslation(body:rootLink():translation() + self.dp)
	    body:calcForwardKinematics()
	    bodyItem:notifyKinematicStateChange()
	    self.dp = -self.dp
	 end
      end

      self.connections:add(
	 self.timer:sigTimeout():connect(self.onTimeout))

      cnoid.MainWindow.instance():addToolBar(self.toolBar)

      setmetatable(self, { 
         __gc = function(self)
	    print "cnoid.MainWindow.instance():removeToolBar(self.toolBar)"
	    cnoid.MainWindow.instance():removeToolBar(self.toolBar)
	 end
      })

      return self
   end
}

bodyShaker = BodyShaker.new()
