
import cnoid.Base
import cnoid.BodyPlugin

def pushWaist():
    robotItem = cnoid.Base.RootItem.instance().findItem("World/SR1")
    simulatorItem = cnoid.BodyPlugin.SimulatorItem.findActiveSimulatorItemFor(robotItem)
    waistLink = robotItem.body().link("WAIST")
    simulatorItem.setExternalForce(robotItem, waistLink, [0, 0, 0], [200, 0.0, 0.0], 1)

pushWaist()

# Use 'callLater' to execute the above function in the background execution mode
# coid.Base.callLater(pushWaist)
