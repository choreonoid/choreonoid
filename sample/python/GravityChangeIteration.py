from cnoid.Base import *
from cnoid.BodyPlugin import *

def doNextSimulation():
    global simulationCount, gravity
    if simulationCount < 5:
        print("Simulation trial {}, gravity: {:.1f}".format(simulationCount + 1, gravity))
        ItemTreeView.instance.selectItem(simulatorItem)
        simulatorItem.setGravity([0, 0, -gravity])
        simulatorItem.notifyUpdate()
        simulatorItem.startSimulation()
        simulationCount += 1
        gravity -= 2
    else:
        print("All the simulation trials have been finished.")
        connection.disconnect()
        
simulatorItem = Item.find("AISTSimulator")
simulationCount = 0
gravity = 9.8
connection = simulatorItem.sigSimulationFinished.connect(doNextSimulation)
doNextSimulation()
