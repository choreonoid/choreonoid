
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

bodyitem = bp.BodyItem()
bodyitem.setName("Hello, Choreonoid")
bodyitem.loadModelFile(shared + "/model/GR001/GR001.dae")
bodyitem.moveToOrigin()

gr = bodyitem.body()
print gr
curr = bodyitem.currentBaseLink()
print curr
bodyitem.setCurrentBaseLink(curr)
bodyitem.setPresetPose(bp.PresetPoseID.INITIAL_POSE)
bodyitem.moveToOrigin()
bodyitem.copyKinematicState()
bodyitem.pasteKinematicState()
bodyitem.clearCollisions()

bodyitem.storeInitialState()
bodyitem.restoreInitialState()

bodyitem.beginKinematicStateEdit()
bodyitem.acceptKinematicStateEdit()
bodyitem.undoKinematicState()

bodyitem.enableSelfCollisionDetection(False)
print bodyitem.isSelfCollisionDetectionEnabled()

bodyitem.centerOfMass()
bodyitem.doLegIkToMoveCm(eigen.Vector3(0.0, 0.0, 0.0), True)
bodyitem.zmp()
bodyitem.setZmp(eigen.Vector3(0.0, 0.0, 0.0))
bodyitem.setStance(0.0)

bodyitem.calcForwardKinematics(True, True)
bodyitem.notifyKinematicStateChange(True, True, True)

world = bp.WorldItem()
print world.isCollisionDetectionEnabled()

#world.updateCollisions()
#world.updateCollisionDetector()
#world.selectCollisionDetector("AISTCollisionDetector")
#world.enableCollisionDetection(True)

aist = bp.AISTSimulatorItem()

print aist.isRunning()
print aist.currentFrame()
print aist.worldTimeStep()
aist.setRealtimeSyncMode(True)
aist.setTimeRangeMode(True)
aist.setAllLinkPositionOutputMode(False)
aist.setRecordingEnabled(True)
aist.setDeviceStateOutputEnabled(False)
aist.setDoingSimulationLoop(False)
aist.setActiveControlPeriodOnlyMode(False)
aist.setStopRequested(False)
aist.setNeedToUpdateSimBodyLists(False)
aist.setHasActiveFreeBodies(False)
aist.setDoReset(True)
aist.setWaitingForSimulationToStop(False)
aist.setMaxFrame(500)
aist.setFillLevelId(False)
aist.setActualSimulationTime(30)
aist.setFinishTime(60)
aist.selectMotionItems()

value = util.FloatingNumberString("Test")
aist.setDynamicsMode(False)
aist.setIntegrationMode(False)
aist.setGravity(eigen.Vector3(0.0, 0.0, 9.8))
aist.setStaticFriction(0.0)
aist.setSlipFriction(0.0)
aist.setCullingThresh(value)
aist.setErrorCriterion(value)
aist.setMaxNumIterations(100)
aist.setContactCorrectionDepth(value)
aist.setContactCorrectionVelocityRatio(value)
aist.setEpsilon(0.0)
aist.set2Dmode(False)
aist.setKinematicWalkingEnabled(False)

sim = bp.SimulationBar.instance()
sim.startSimulation(aist, True)
sim.stopSimulation(aist)

temp = util.MultiValueSeq()

motion = bp.BodyMotionItem()
#print motion.jointPosSeq()
#print motion.linkPosSeq()

print motion.numExtraSeqItems()
#print motion.extraSeqKey(0)
motion.updateExtraSeqItems()
