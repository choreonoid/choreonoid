import os
import cnoid.Base

def loadProject(worldProject, simulatorProject, robotProject):

    directory = os.path.dirname(os.path.realpath(__file__))
    
    pm = cnoid.Base.ProjectManager.instance

    pm.loadProject(os.path.join(directory, worldProject + ".cnoid"))

    worldItem = cnoid.Base.Item.find("World")

    pm.loadProject(os.path.join(directory, simulatorProject + ".cnoid"), worldItem)

    pm.loadProject(os.path.join(directory, robotProject + ".cnoid"), worldItem)

    pm.setCurrentProjectName(worldProject + "-" + robotProject)
