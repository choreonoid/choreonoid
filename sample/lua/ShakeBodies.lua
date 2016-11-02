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

      --[[

      local bodyItems = cnoid.BodyItemList.new()
      local dp = cnoid.Vector3(0.0, 0.0, 0.01)
      local connections = cnoid.ScopedConnectionSet.new()

      function onSelectionChanged(items)
	 bodyItems = cnoid.BodyItemList(items)
	 if not bodyItems:empty() and button:isChecked() then
	    timer:start(50)
	 else
	    timer:stop()
	 end
      end

      button.sigToggled().connect(onSelectionChanged)

      local timer = cnoid.Timer.new()
      connections:add(timer:sigTimeout():connect(
         function()
	    for i, bodyItem in ipairs(bodyItems) do
	       local body = bodyItem:body()
	       body:rootLink():setTranslation(body:rootLink():translation() + dp)
	       body:calcForwardKinematics()
	       bodyItem:notifyKinematicStateChange()
	       dp = -dp
	    end
	 end
         ))

      connections.add(cnoid.ItemTreeView.instance():sigSelectionChanged().connect(onSelectionChanged))

      ]]--

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
