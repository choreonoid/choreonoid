cnoid.require "Util"
cnoid.require "Base"
cnoid.require "Body"
cnoid.require "BodyPlugin"

function callback(on)
   print("Button is ", (on and "on" or "off"))
end

toolBar = cnoid.ToolBar.new("Test")
toolBar:setVisibleByDefault(true)
button = toolBar:addToggleButton("Test")
button:setChecked(true)
signal = button:sigToggled()
connection = signal:connect(callback)
cnoid.MainWindow.instance():addToolBar(toolBar)
