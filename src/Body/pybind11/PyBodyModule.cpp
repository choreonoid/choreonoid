/*!
  @author Shin'ichiro Nakaoka
 */

#include "../Body.h"
#include "../BodyLoader.h"
#include "../BodyMotion.h"
#include "../InverseKinematics.h"
#include "../JointPath.h"
#include <cnoid/ValueTree>
#include <cnoid/SceneGraph>
#include <cnoid/PyUtil>
#include <pybind11/operators.h>

namespace py = pybind11;
using namespace cnoid;

namespace
{
Position Link_get_position(Link& self) { return self.position(); }
void Link_set_position(Link& self, const Position& T) { self.position() = T; }
Vector3 Link_get_translation(Link& self) { return self.translation(); }
void Link_set_translation(Link& self, const Vector3& p) { self.translation() = p; }
Matrix3 Link_get_rotation(Link& self) { return self.rotation(); }
void Link_set_rotation(Link& self, const Matrix3& R) { self.rotation() = R; }
Position Link_get_Tb(Link& self) { return self.Tb(); }
void Link_set_Tb(Link& self, const Position& T) { self.Tb() = T; }
Vector3 Link_get_offsetTranslation(Link& self) { return self.offsetTranslation(); }
Matrix3 Link_get_offsetRotation(Link& self) { return self.offsetRotation(); }
double Link_get_q(Link& self) { return self.q(); }
void Link_set_q(Link& self, double q) { self.q() = q; }
double Link_get_dq(Link& self) { return self.dq(); }
void Link_set_dq(Link& self, double dq) { self.dq() = dq; }
double Link_get_ddq(Link& self) { return self.ddq(); }
void Link_set_ddq(Link& self, double ddq) { self.ddq() = ddq; }
double Link_get_u(Link& self) { return self.u(); }
void Link_set_u(Link& self, double u) { self.u() = u; }
Vector3 Link_get_v(Link& self) { return self.v(); }
void Link_set_v(Link& self, const Vector3& v) { self.v() = v; }
Vector3 Link_get_w(Link& self) { return self.w(); }
void Link_set_w(Link& self, const Vector3& w) { self.w() = w; }
Vector3 Link_get_dv(Link& self) { return self.dv(); }
void Link_set_dv(Link& self, const Vector3& dv) { self.dv() = dv; }
Vector3 Link_get_dw(Link& self) { return self.dw(); }
void Link_set_dw(Link& self, const Vector3& dw) { self.dw() = dw; }
Vector3 Link_get_wc(Link& self) { return self.wc(); }
void Link_set_wc(Link& self, const Vector3& wc) { self.wc() = wc; }
Vector6 Link_get_F_ext(Link& self) { return self.F_ext(); }
void Link_set_F_ext(Link& self, const Vector6& F) { self.F_ext() = F; }
Vector3 Link_get_f_ext(Link& self) { return self.f_ext(); }
void Link_set_f_ext(Link& self, const Vector3& f) { self.f_ext() = f; }
Vector3 Link_get_tau_ext(Link& self) { return self.tau_ext(); }
void Link_set_tau_ext(Link& self, const Vector3& tau) { self.tau_ext() = tau; }

MultiValueSeqPtr BodyMotion_get_jointPosSeq(BodyMotion& self) { return self.jointPosSeq(); }
void BodyMotion_set_jointPosSeq(BodyMotion& self, const MultiValueSeqPtr& jointPosSeq) { self.jointPosSeq() = jointPosSeq; }
MultiSE3SeqPtr BodyMotion_get_linkPosSeq(BodyMotion& self) { return self.linkPosSeq(); }
void BodyMotion_set_linkPosSeq(BodyMotion& self, const MultiSE3SeqPtr& linkPosSeq) { self.linkPosSeq() = linkPosSeq; }

Position Device_get_T_local(Device& self) { return (Position) self.T_local(); }
void Device_set_T_local(Device& self, const Position& T_local) { self.T_local() = T_local.matrix(); }

} // namespace

namespace cnoid 
{
PYBIND11_PLUGIN(Body)
{
    py::module m("Body", "Body Python Module");

    py::module::import("cnoid.Util");

    py::class_< Link, LinkPtr, Referenced> link(m, "Link");
    link
        .def("index", &Link::index)
        .def("isValid", &Link::isValid)
        .def("parent", [](Link& self){ return LinkPtr(self.parent()); } )
        .def("sibling", [](Link& self){ return LinkPtr(self.sibling()); } )
        .def("child", [](Link& self){ return LinkPtr(self.child()); } )
        .def("isRoot", &Link::isRoot)
        .def_property("T", Link_get_position, Link_set_position)
        .def("position", Link_get_position)
        .def("setPosition", Link_set_position)
        .def_property("p", Link_get_translation, Link_set_translation)
        .def("translation", Link_get_translation)
        .def("setTranslation", Link_set_translation)
        .def_property("R", Link_get_rotation, Link_set_rotation)
        .def("rotation", Link_get_rotation)
        .def("setRotation", Link_set_rotation)
        .def_property("Tb", Link_get_Tb, Link_set_Tb)
        .def("b", Link_get_offsetTranslation)
        .def("offsetTranslation", Link_get_offsetTranslation)
        .def_property_readonly("Rb", Link_get_offsetRotation)
        .def("offsetRotation", Link_get_offsetRotation)
        .def("jointId", &Link::jointId)
        .def("jointType", &Link::jointType)
        .def("isFixedJoint", &Link::isFixedJoint)
        .def("isFreeJoint", &Link::isFreeJoint)
        .def("isRotationalJoint", &Link::isRotationalJoint)
        .def("isSlideJoint", &Link::isSlideJoint)
        .def_property_readonly("a", &Link::a, py::return_value_policy::copy)
        .def("jointAxis", &Link::jointAxis, py::return_value_policy::copy)
        .def_property_readonly("d", &Link::d, py::return_value_policy::copy)
        .def_property("q", Link_get_q, Link_set_q)
        .def_property("dq", Link_get_dq, Link_set_dq)
        .def_property("ddq", Link_get_ddq, Link_set_ddq)
        .def_property("u", Link_get_u, Link_set_u)
        .def_property_readonly("q_upper", &Link::q_upper)
        .def_property_readonly("q_lower", &Link::q_lower)
        .def_property_readonly("dq_upper", &Link::dq_upper)
        .def_property_readonly("dq_lower", &Link::dq_lower)
        .def_property("v", Link_get_v, Link_set_v)
        .def_property("w", Link_get_w, Link_set_w)
        .def_property("dv", Link_get_dv, Link_set_dv)
        .def_property("dw", Link_get_dw, Link_set_dw)
        .def_property_readonly("c", &Link::c, py::return_value_policy::copy)
        .def("centerOfMass", &Link::centerOfMass, py::return_value_policy::copy)
        .def_property("wc", Link_get_wc, Link_set_wc)
        .def("centerOfMassGlobal", &Link::centerOfMassGlobal, py::return_value_policy::copy)
        .def_property_readonly("m", &Link::m)
        .def("mass", &Link::mass)
        .def_property_readonly("I", &Link::I, py::return_value_policy::copy)
        .def_property_readonly("Jm2", &Link::Jm2)
        .def_property("F_ext", Link_get_F_ext, Link_set_F_ext)
        .def_property("f_ext", Link_get_f_ext, Link_set_f_ext)
        .def_property("tau_ext", Link_get_tau_ext, Link_set_tau_ext)
        .def("name", &Link::name, py::return_value_policy::reference)
        .def("visualShape", [](const Link& self) { return SgNodePtr(self.visualShape()); })
        .def("collisionShape",  [](const Link& self) { return SgNodePtr(self.collisionShape()); })
        .def("setIndex", &Link::setIndex)
        .def("prependChild", &Link::prependChild)
        .def("appendChild", &Link::appendChild)
        .def("removeChild", &Link::removeChild)
        .def("setJointType", &Link::setJointType)
        .def("setJointId", &Link::setJointId)
        .def("setJointAxis", &Link::setJointAxis)
        .def("setJointRange", &Link::setJointRange)
        .def("setJointVelocityRange", &Link::setJointVelocityRange)
        .def("setMass", &Link::setMass)
        .def("setInertia", &Link::setInertia)
        .def("setCenterOfMass", &Link::setCenterOfMass)
        .def("setEquivalentRotorInertia", &Link::setEquivalentRotorInertia)
        .def("setName", &Link::setName)
        .def("setVisualShape", &Link::setVisualShape)
        .def("setCollisionShape", &Link::setCollisionShape)
        .def("attitude", &Link::attitude)
        .def("setAttitude", &Link::setAttitude)
        .def("calcRfromAttitude", &Link::calcRfromAttitude)
        .def("info", [](Link& self) { return MappingPtr(self.info()); })
        .def("info", [](Link& self, const std::string& key, py::object defaultValue) {
            if(!PyFloat_Check(defaultValue.ptr())){
                PyErr_SetString(PyExc_TypeError, "The argument type is not supported");
                throw py::error_already_set();
            }
            double v = defaultValue.cast<double>();
            return py::cast(self.info(key, v));
        })
        .def("floatInfo", [](Link& self, const std::string& key) { return self.info<double>(key); })
        ;

    py::enum_<Link::JointType>(link, "JointType")
        .value("ROTATIONAL_JOINT", Link::JointType::ROTATIONAL_JOINT)
        .value("SLIDE_JOINT", Link::JointType::SLIDE_JOINT)
        .value("FREE_JOINT", Link::JointType::FREE_JOINT)
        .value("FIXED_JOINT", Link::JointType::FIXED_JOINT)
        .value("CRAWLER_JOINT", Link::JointType::CRAWLER_JOINT)
        .export_values();

    py::class_< Body, BodyPtr, Referenced> body(m, "Body");
    body
        .def("clone", [](Body& self) { return BodyPtr(self.clone()); })
        .def("createLink", [](Body& self, const Link* org) {
            return self.createLink(org); }, py::arg("org")=0 )
        .def("name", &Body::name, py::return_value_policy::reference)
        .def("setName", &Body::setName)
        .def("modelName", &Body::modelName, py::return_value_policy::reference)
        .def("setModelName", &Body::setModelName)
        .def("setRootLink", &Body::setRootLink)
        .def("updateLinkTree", &Body::updateLinkTree)
        .def("initializeState", &Body::initializeState)
        .def("numJoints", &Body::numJoints)
        .def("numVirtualJoints", &Body::numVirtualJoints)
        .def("numAllJoints", &Body::numAllJoints)
        .def("joint", [](Body& self, int id) { return LinkPtr(self.joint(id)); })
        .def("numLinks", &Body::numLinks)
        .def("link", [](Body& self, int index) { return LinkPtr(self.link(index)); })
        .def("link", [](Body& self, const std::string& name) { return LinkPtr(self.link(name)); })
        .def("rootLink",[](Body& self) { return LinkPtr(self.rootLink()); })
        .def("numDevices", &Body::numDevices)
        .def("device", [](Body& self, int index) { return DevicePtr(self.device(index)); })
        .def("addDevice", &Body::addDevice)
        .def("initializeDeviceStates", &Body::initializeDeviceStates)
        .def("clearDevices", &Body::clearDevices)
        .def("isStaticModel", &Body::isStaticModel)
        .def("isFixedRootModel", &Body::isFixedRootModel)
        .def("resetDefaultPosition", &Body::resetDefaultPosition)
        .def("defaultPosition", &Body::defaultPosition, py::return_value_policy::copy)
        .def("mass", &Body::mass)
        .def("calcCenterOfMass", &Body::calcCenterOfMass, py::return_value_policy::copy)
        .def("centerOfMass", &Body::centerOfMass, py::return_value_policy::copy)
        .def("calcTotalMomentum", [](Body& self) {
            Vector3 P, L;
            self.calcTotalMomentum(P, L);
            py::list p, l;
            for(int i=0; i < 3; ++i){
                p.append(P[i]);
                l.append(L[i]);
            }
            return py::make_tuple(p, l);
        })
        .def("calcForwardKinematics", &Body::calcForwardKinematics,
                py::arg("calcVelocity")=false, py::arg("calcAcceleration")=false)
        .def("clearExternalForces", &Body::clearExternalForces)
        .def("numExtraJoints", &Body::numExtraJoints)
        //.def("extraJoint", extraJoint, , return_value_policy<reference_existing_object>())
        //.def("addExtraJoint", &Body::addExtraJoint)
        .def("clearExtraJoints", &Body::clearExtraJoints)
        //.def("installCustomizer", installCustomizer)
        .def("hasVirtualJointForces", &Body::hasVirtualJointForces)
        .def("setVirtualJointForces", &Body::setVirtualJointForces)
        .def_static("addCustomizerDirectory", &Body::addCustomizerDirectory)
        .def(BodyMotion::Frame() >> py::self)
        ;

    py::class_< ExtraJoint > extraJoint(m, "ExtraJoint");
    extraJoint
        .def(py::init<>())
        .def(py::init<ExtraJoint::ExtraJointType, const Vector3&>())
        .def("setType", &ExtraJoint::setType)
        .def("setAxis", &ExtraJoint::setAxis)
        .def("setPoint", &ExtraJoint::setPoint)
        ;

    py::enum_<ExtraJoint::ExtraJointType>(extraJoint, "ExtraJointType")
        .value("EJ_PISTON", ExtraJoint::ExtraJointType::EJ_PISTON)
        .value("EJ_BALL", ExtraJoint::ExtraJointType::EJ_BALL)
        .export_values();

    py::class_<AbstractBodyLoader>(m, "AbstractBodyLoader")
        .def("setVerbose", &AbstractBodyLoader::setVerbose)
        .def("setShapeLoadingEnabled", &AbstractBodyLoader::setShapeLoadingEnabled)
        .def("setDefaultDivisionNumber", &AbstractBodyLoader::setDefaultDivisionNumber)
        .def("setDefaultCreaseAngle", &AbstractBodyLoader::setDefaultCreaseAngle)
        .def("load", &AbstractBodyLoader::load)
        ;

    py::class_<BodyLoader, AbstractBodyLoader>(m, "BodyLoader")
        .def("load", [](BodyLoader& self, const std::string& filename) { return BodyPtr(self.load(filename)); })
        .def("lastActualBodyLoader", &BodyLoader::lastActualBodyLoader)
        ;

    py::class_< JointPath, JointPathPtr>(m, "JointPath")
        .def(py::init<>())
        .def("numJoints", &JointPath::numJoints)
        .def("joint", [](JointPath& self, int index) { return LinkPtr(self.joint(index)); })
        .def("baseLink", [](JointPath& self) { return LinkPtr(self.baseLink()); })
        .def("endLink", [](JointPath& self) { return LinkPtr(self.endLink()); })
        .def("indexOf", &JointPath::indexOf)
        .def("customizeTarget", &JointPath::customizeTarget)
        .def("numIterations", &JointPath::numIterations)
        .def("calcJacobian", &JointPath::calcJacobian)
        .def("calcInverseKinematics", (bool (JointPath::*)(const Vector3& , const Matrix3&))
                &JointPath::calcInverseKinematics)
        .def("calcInverseKinematics", (bool (JointPath::*)(const Vector3&, const Matrix3&, const Vector3&, const Matrix3&))
                &JointPath::calcInverseKinematics)
        ;

    m.def("getCustomJointPath", getCustomJointPath);

    py::class_< BodyMotion, BodyMotionPtr> bodyMotion(m, "BodyMotion");
    bodyMotion
        .def("setNumParts", &BodyMotion::setNumParts)
        .def("getNumParts", &BodyMotion::getNumParts)
        .def("numJoints", &BodyMotion::numJoints)
        .def("numLings", &BodyMotion::numLinks)
        .def("frameRate", &BodyMotion::frameRate)
        .def("getFrameRate",&BodyMotion::getFrameRate)
        .def("setFrameRate", &BodyMotion::setFrameRate)
        .def("getOffsetTimeFrame", &BodyMotion::getOffsetTimeFrame)
        .def("numFrames", &BodyMotion::numFrames)
        .def("getNumFrames", &BodyMotion::getNumFrames)
        .def("setNumFrames", &BodyMotion::setNumFrames)
        //.def("jointPosSeq", BodyMotion_jointPosSeq, return_value_policy<reference_existing_object>())
        //.def("linkPosSeq", BodyMotion_linkPosSeq, return_value_policy<reference_existing_object>());
        .def_property("jointPosSeq", BodyMotion_get_jointPosSeq, BodyMotion_set_jointPosSeq)
        .def_property("linkPosSeq", BodyMotion_get_linkPosSeq, BodyMotion_set_linkPosSeq)
        .def("frame", (BodyMotion::Frame (BodyMotion::*)(int)) &BodyMotion::frame)
        ;

    py::class_< BodyMotion::Frame >(m, "Frame")
        .def("frame", &BodyMotion::Frame::frame)
        .def(py::self << Body())
        .def(Body() >> py::self)
        ;

    py::class_< Device, DevicePtr>(m, "Device")
        .def("setIndex", &Device::setIndex)
        .def("setId", &Device::setId)
        .def("setName", &Device::setName)
        .def("setLink", &Device::setLink)
        .def("clone", [](Device& self) { return DevicePtr(self.clone()); })
        .def("clearState", &Device::clearState)
        .def("hasStateOnly", &Device::hasStateOnly)
        .def("index", &Device::index)
        .def("id", &Device::id)
        .def("name", &Device::name, py::return_value_policy::reference)
        .def("link", [](Device& self) { return LinkPtr(self.link()); })
        .def_property("T_local", Device_get_T_local, Device_set_T_local)
        ;

#ifdef _MSC_VER
    register_ptr_to_python<BodyPtr>();
    register_ptr_to_python<LinkPtr>();
#endif

    return m.ptr();
}

}; // namespace cnoid
