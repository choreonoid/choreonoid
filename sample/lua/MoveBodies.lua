cnoid.require "Util"
cnoid.require "Base"
cnoid.require "Body"
cnoid.require "BodyPlugin"

bodyItems = cnoid.RootItem.instance():getDescendantItems(cnoid.BodyItem)

for i, bodyItem in ipairs(bodyItems) do
   body = bodyItem:body()
   rootLink = body:rootLink()
   for i=1,20 do
      rootLink:setTranslation(rootLink:translation() + cnoid.Vector3(0, 0, 0.01))
      body:calcForwardKinematics()
      bodyItem:notifyKinematicStateChange()
      cnoid.MessageView.instance():flush()
      cnoid.sleep(0.01)
   end
end
