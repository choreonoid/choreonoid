cnoid.require "Base"
cnoid.require "BodyPlugin"
posix = require "posix"

bodyItems = cnoid.RootItem.instance():getDescendantItems(cnoid.BodyItem)

for bodyItem in bodyItems do
   body = bodyItem:body()
   rootLink = body:rootLink()
   for i=1,20 do
      rootLink:p += { 0, 0, 0.01 }
      body:calcForwardKinematics()
      bodyItem:notifyKinematicStateChange()
      coid.MessageView.instance():flush()
      time.sleep(0.01)
   end
end
