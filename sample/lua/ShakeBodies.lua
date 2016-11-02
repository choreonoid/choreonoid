cnoid.require "Util"
cnoid.require "Base"
cnoid.require "Body"
cnoid.require "BodyPlugin"

BodyShaker = {
   new = function()
      local self = { }
      self.toolBar = cnoid.ToolBar.new("ShakeBar")
      self.toolBar:setVisibleByDefault(true)
      local button = self.toolBar:addToggleButton("Shake")
      button:setChecked(true)

      local bodyItems = { }
      local dp = cnoid.Vector3(0.0, 0.0, 0.01)
      local connections = cnoid.ScopedConnectionSet.new()

      function onSelectionChanged(items)
	 bodyItems = cnoid.extractBodyItems(items)
	 if #bodyItems > 0 and button:isChecked() then
	    timer:start(50)
	 else
	    timer:stop()
	 end
      end

      connections.add(
	 cnoid.ItemTreeView.instance():sigSelectionChanged():connect(onSelectionChanged))

      connections:add(
	 button:sigToggled():connect(
	    function(on)
	       onSelectionChanged(cnoid.ItemTreeView.instance():selectedItems())
	    end))

      local timer = cnoid.Timer.new()
      connections:add(
	 timer:sigTimeout():connect(
	    function()
	       for i, bodyItem in ipairs(bodyItems) do
		  local body = bodyItem:body()
		  body:rootLink():setTranslation(body:rootLink():translation() + dp)
		  body:calcForwardKinematics()
		  bodyItem:notifyKinematicStateChange()
		  dp = -dp
	       end
	    end))

      print "cnoid.MainWindow.instance():addToolBar(toolBar)"
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
