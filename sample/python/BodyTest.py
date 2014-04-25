
# @author Hisashi Ikari

import sys
import cnoid.Util as util
import cnoid.Base as base
import cnoid.Body as body
import cnoid.BodyPlugin as bp
import cnoid.SimpleControllerPlugin as simple
import minieigen as eigen

hrp = bp.BodyItem()
hrp.setName("HRP4C")
shared = util.shareDirectory()
hrp.loadModelFile(shared + "/model/ext/HRP4C/HRP4C.dae")
hrp.setZmp(eigen.Vector3(0.0, 0.0, 0.0))

hrpbody = hrp.body()
hrplink = hrpbody.createLink(hrpbody.rootLink())
print hrplink

print hrpbody.name()
print hrpbody.modelName()
hrpbody.setName("HRP4C!")
hrpbody.setModelName("HRP4C!!!")
hrproot = hrpbody.rootLink()

print hrproot
print hrproot.name()

hrpbody.updateLinkTree()
hrpbody.initializeState()
print hrpbody.numJoints()
print hrpbody.numVirtualJoints()
print hrpbody.numLinks()
print hrpbody.numDevices()

print "Step1 finished."

link = hrpbody.link("WAIST")
link = hrpbody.link(0)

print link.index()
print link.isValid()
print link.parent()
print link.sibling()
print link.child()
print link.isRoot()

print "Step2 finished."

print link.jointAxis()
print link.position()
print link.translation()
print link.rotation()
print link.offsetTranslation()
print link.offsetRotation()
print link.Rs()

print "Step3 finished."

print link.jointId()
print link.jointType()
print link.isFixedJoint()
print link.isFreeJoint()
print link.isSlideJoint()
print link.jointAxis()

print link.q()
link.q(10.0)
print link.dq()
print link.ddq()
print link.u()
link.u(10.0)
print link.q_upper()
print link.q_lower()
print link.dq_upper()
print link.dq_lower()

print link.v()
link.v(eigen.Vector3(1,1,1))
print link.w()
link.w(eigen.Vector3(1,1,1))
print link.dv()
link.dv(eigen.Vector3(1,1,1))
print link.dw()
link.dw(eigen.Vector3(1,1,1))

print link.centerOfMass()
print link.centerOfMassGlobal()
print link.mass()
print link.I()
print link.Jm2()
print link.F_ext()

print link.name()
print link.setIndex(0)
print link.prependChild(link)
print link.appendChild(link)
print link.removeChild(link)
link.setName("WAIST!")

device = hrpbody.device(0)
hrpbody.addDevice(device)
hrpbody.initializeDeviceStates()
hrpbody.clearDevices()

print hrpbody.defaultPosition()
print hrpbody.isStaticModel()
print hrpbody.isFixedRootModel()
print hrpbody.mass()
print hrpbody.centerOfMass()

hrpbody.calcForwardKinematics(True, True)
hrpbody.clearExternalForces()

print hrpbody.numExtraJoints()
hrpbody.clearExtraJoints()
hrpbody.installCustomizer()

print hrpbody.hasVirtualJointForces()
hrpbody.addCustomizerDirectory("")

traverse = body.LinkTraverse()
print traverse.numLinks()
print traverse.empty()
print traverse.size()
print traverse.isDownward(0)
print traverse.rootLink()

collada = body.ColladaBodyLoader()

collada.format()
collada.setVerbose(False)
collada.setShapeLoadingEnabled(False)
collada.setDefaultDivisionNumber(0)
collada.setDefaultCreaseAngle(3.14)

print "finished."
