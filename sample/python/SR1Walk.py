#!/usr/bin/env python
# -*- coding: utf-8 -*-

# @author Hisashi Ikari

import cnoid.Util as util
import cnoid.Base as base
import cnoid.Body as body
import cnoid.BodyPlugin as bp
import cnoid.SimpleControllerPlugin as simple
import minieigen as eigen

shared = util.shareDirectory()
top = util.executableTopDirectory()

root = base.RootItem.instance()
view = base.ItemTreeView.instance()

world = bp.WorldItem()
world.setName("World")
root.addChildItem(world)

time = base.TimeBar.instance()
time.setFillLevelSyncCheck(False)
time.setFrameRate(500)
time.setTimeRange(0.0, 15.0)

scene = base.SceneView.instance()
scene.setHeadLight(True)
scene.setFloorGrid(True)
scene.setWorldLightIntensity(0.55)
scene.setWorldLightAmbient(0.5)
scene.setBackgroundColor(eigen.Vector3(0.1, 0.1, 0.3))

scene.setCameraPosition(
	eigen.Vector3(2.38053, 1.54674, 1.10801),
	eigen.Vector3(-0.82064, -0.55381, -0.140871),
	eigen.Vector3(-0.116769, -0.0788014, 0.990028))
scene.setFieldOfView(0.6978);
scene.setNear(0.01);
scene.setFar(10000.0);
scene.setHeight(20);

sr1 = bp.BodyItem()
sr1.setName("SR1")
sr1.loadModelFile(shared + "/model/SR1/SR1.yaml")
sr1.setZmp(eigen.Vector3(0.0, 0.0, 0.0))

body = sr1.body()
link = body.rootLink()

link.p(eigen.Vector3(0.0, 0.0, 0.7135))

link.R(eigen.Matrix3(
	1.0, 0.0, 0.0,    
	0.0, 1.0, 0.0,
	0.0, 0.0, 1.0))

pos = [0.000000, -0.036652,  0.000000,  0.078540, -0.041888,  
       0.000000,  0.174533, -0.003491,  0.000000, -1.570796,
       0.000000,  0.000000,  0.000000,  0.000000, -0.036652,
       0.000000,  0.078540, -0.041888,  0.000000,  0.174533, 
      -0.003491,  0.000000, -1.570796,  0.000000,  0.000000,  
       0.000000,  0.000000,  0.000000,  0.000000 ]

for i in range(body.numJoints()):
	body.joint(i).q(pos[i])

sr1.setPresetPose(bp.PresetPoseID.INITIAL_POSE)

world.addChildItem(sr1)
view.checkItem(sr1)

floor = bp.BodyItem()
floor.setName("Floor")
floor.loadModelFile(shared + "/model/misc/floor.wrl")
world.addChildItem(floor)

aistsim = bp.AISTSimulatorItem()
aistsim.setName("AISTSimulatorItem")
world.addChildItem(aistsim)

cont = simple.SimpleControllerItem()
cont.setName("SR1WalkController")
sr1.addChildItem(cont)
cont.setControllerDllName(top + "/lib/choreonoid-1.5/simplecontroller/SR1WalkPatternController")

view.selectItem(aistsim)
sim = bp.SimulationBar.instance()
sim.startSimulation(aistsim)
