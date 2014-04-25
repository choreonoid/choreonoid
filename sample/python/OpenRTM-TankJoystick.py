#!/usr/bin/env python
# -*- coding: utf-8 -*-

# @author Hisashi Ikari

import sys
import cnoid.Util as util
import cnoid.Base as base
import cnoid.Body as body
import cnoid.BodyPlugin as bp
import cnoid.OpenRTMPlugin as rtm
import cnoid.Corba as corba
import minieigen as eigen
import RTC
import OpenRTM_aist

from omniORB import CORBA

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

aistsim = bp.AISTSimulatorItem()
aistsim.setName("AISTSimulatorItem")
aistsim.setRealtimeSyncMode(True)
aistsim.setTimeRangeMode(bp.TimeRangeMode.UNLIMITED)
world.addChildItem(aistsim)

cont = rtm.BodyRTCItem()
cont.setName("TankControllerRTC")
tank.addChildItem(cont)
cont.setModuleName(top + "/lib/choreonoid-1.5/rtc/TankJoystickControllerRTC")

joy = rtm.RTCItem()
joy.setName("JoystickControllerRTC")
world.addChildItem(joy)
joy.setModuleName(top + "/lib/choreonoid-1.5/rtc/JoystickRTC")


orb = corba.getORB()

naming = OpenRTM_aist.CorbaNaming(orb, "localhost")

controller = OpenRTM_aist.CorbaConsumer()
robot = OpenRTM_aist.CorbaConsumer()
joystick = OpenRTM_aist.CorbaConsumer()

controller.setObject(naming.resolve("TankJoystickControllerRTC0.rtc"))
inobj = controller.getObject()._narrow(RTC.RTObject)

robot.setObject(naming.resolve("Tank.rtc"))
outobj = robot.getObject()._narrow(RTC.RTObject)

joystick.setObject(naming.resolve("JoystickRTC0.rtc"))
joyobj = joystick.getObject()._narrow(RTC.RTObject)

subs_type = "flush"
data_type = "IDL:RTC/TimedDoubleSeq:1.0"

pcont = inobj.get_ports()
for i in range(0, 4):
    pcont[i].disconnect_all()

probot = outobj.get_ports()
for i in range(0, 3):
    probot[i].disconnect_all()

pjoy = joyobj.get_ports()
for i in range(0, 2):
    pjoy[i].disconnect_all()

# connect ports(u to u)
conprof1 = RTC.ConnectorProfile("connector0", "", [pcont[3],probot[2]], [])

OpenRTM_aist.CORBA_SeqUtil.push_back(conprof1.properties,
                                   OpenRTM_aist.NVUtil.newNV("dataport.interface_type",
                                                             "corba_cdr"))
OpenRTM_aist.CORBA_SeqUtil.push_back(conprof1.properties,
                                   OpenRTM_aist.NVUtil.newNV("dataport.dataflow_type",
                                                             "push"))
OpenRTM_aist.CORBA_SeqUtil.push_back(conprof1.properties,
                                   OpenRTM_aist.NVUtil.newNV("dataport.subscription_type",
                                                             subs_type))
OpenRTM_aist.CORBA_SeqUtil.push_back(conprof1.properties,
                                   OpenRTM_aist.NVUtil.newNV("dataport.data_type",
                                                             data_type))
ret,conprof = probot[0].connect(conprof1)

# connect ports(q to q)
conprof2 = RTC.ConnectorProfile("connector0", "", [pcont[0],probot[0]], [])

OpenRTM_aist.CORBA_SeqUtil.push_back(conprof2.properties,
                                   OpenRTM_aist.NVUtil.newNV("dataport.interface_type",
                                                             "corba_cdr"))
OpenRTM_aist.CORBA_SeqUtil.push_back(conprof2.properties,
                                   OpenRTM_aist.NVUtil.newNV("dataport.dataflow_type",
                                                             "push"))
OpenRTM_aist.CORBA_SeqUtil.push_back(conprof2.properties,
                                   OpenRTM_aist.NVUtil.newNV("dataport.subscription_type",
                                                             subs_type))
OpenRTM_aist.CORBA_SeqUtil.push_back(conprof2.properties,
                                   OpenRTM_aist.NVUtil.newNV("dataport.data_type",
                                                             data_type))
ret,conprof = probot[0].connect(conprof2)

# connect ports(axis to axis)
conprof3 = RTC.ConnectorProfile("connector0", "", [pcont[1],pjoy[0]], [])

OpenRTM_aist.CORBA_SeqUtil.push_back(conprof3.properties,
                                   OpenRTM_aist.NVUtil.newNV("dataport.interface_type",
                                                             "corba_cdr"))
OpenRTM_aist.CORBA_SeqUtil.push_back(conprof3.properties,
                                   OpenRTM_aist.NVUtil.newNV("dataport.dataflow_type",
                                                             "push"))
OpenRTM_aist.CORBA_SeqUtil.push_back(conprof3.properties,
                                   OpenRTM_aist.NVUtil.newNV("dataport.subscription_type",
                                                             subs_type))
OpenRTM_aist.CORBA_SeqUtil.push_back(conprof3.properties,
                                   OpenRTM_aist.NVUtil.newNV("dataport.data_type",
                                                             data_type))
ret,conprof = pcont[0].connect(conprof3)

# connect ports(button to button)
conprof4 = RTC.ConnectorProfile("connector0", "", [pcont[2],pjoy[1]], [])

OpenRTM_aist.CORBA_SeqUtil.push_back(conprof4.properties,
                                   OpenRTM_aist.NVUtil.newNV("dataport.interface_type",
                                                             "corba_cdr"))
OpenRTM_aist.CORBA_SeqUtil.push_back(conprof4.properties,
                                   OpenRTM_aist.NVUtil.newNV("dataport.dataflow_type",
                                                             "push"))
OpenRTM_aist.CORBA_SeqUtil.push_back(conprof4.properties,
                                   OpenRTM_aist.NVUtil.newNV("dataport.subscription_type",
                                                             subs_type))
OpenRTM_aist.CORBA_SeqUtil.push_back(conprof4.properties,
                                   OpenRTM_aist.NVUtil.newNV("dataport.data_type",
                                                             data_type))
ret,conprof = pcont[0].connect(conprof4)

view.selectItem(aistsim)
sim = bp.SimulationBar.instance()
sim.startSimulation(aistsim)
