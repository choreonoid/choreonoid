Base = require "cnoid.Base"
BodyPlugin = require "cnoid.BodyPlugin"
posix = require "posix"

bodyItems = Base.RootItem.instance():getDescendantItems(BodyPlugin.BodyItem)

for bodyItem in bodyItems do
   body = bodyItem:body()
   rootLink = body:rootLink()
   for i=1,20 do
      rootLink:p += { 0, 0, 0.01 }
      body:calcForwardKinematics()
      bodyItem:notifyKinematicStateChange()
      MessageView.instance():flush()
      time.sleep(0.01)
   end
end
