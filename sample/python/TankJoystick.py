#!/usr/bin/env python
# -*- coding: utf-8 -*-

# @author Hisashi Ikari

import sys
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
time.setFrameRate(1000)
time.setPlaybackFrameRate(100)
time.setTimeRange(0.0, 5.0)

scene = base.SceneView.instance()
scene.setHeadLight(False)
scene.setFloorGrid(False)
scene.setWorldLightIntensity(0.1)
scene.setWorldLightAmbient(0.0)
scene.setBackgroundColor(eigen.Vector3(0.0, 0.0, 0.0))

scene.setCameraPosition(
	eigen.Vector3(-2.86824, 6.25331, 2.49127),
	eigen.Vector3(0.412288, -0.847325, -0.334751),
	eigen.Vector3(0.146464, -0.301009, 0.942307))
scene.setFieldOfView(0.6978);
scene.setNear(0.01);
scene.setFar(10000.0);
scene.setHeight(20);

labo = bp.BodyItem()
labo.setName("Labo1")
labo.loadModelFile(shared + "/model/Labo1/Labo1.wrl")
world.addChildItem(labo)
view.checkItem(labo)

tank = bp.BodyItem()
tank.setName("Tank")
tank.loadModelFile(shared + "/model/misc/tank.wrl")
world.addChildItem(tank)
view.checkItem(tank)

cont = simple.SimpleControllerItem()
cont.setName("JoystickControlller")
tank.addChildItem(cont)
cont.setControllerDllName(top + "/lib/choreonoid-1.5/simplecontroller/TankJoystickController")

aistsim = bp.AISTSimulatorItem()
aistsim.setName("AISTSimulatorItem")
aistsim.setRealtimeSyncMode(True)
aistsim.setTimeRangeMode(bp.TimeRangeMode.UNLIMITED)
world.addChildItem(aistsim)

view.selectItem(aistsim)
sim = bp.SimulationBar.instance()
sim.startSimulation(aistsim)
