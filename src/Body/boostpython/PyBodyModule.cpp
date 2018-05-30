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

using namespace cnoid;
namespace py = boost::python;

// for MSVC++2015 Update3
CNOID_PYTHON_DEFINE_GET_POINTER(Link)
CNOID_PYTHON_DEFINE_GET_POINTER(Body)

namespace
{

LinkPtr Link_parent(Link& self) { return self.parent(); }
LinkPtr Link_child(Link& self) { return self.child(); }
LinkPtr Link_sibling(Link& self) { return self.sibling(); }
Position Link_get_position(Link& self) { return self.position(); }
void Link_set_position(Link& self, const Position& T) { self.position() = T; }
Vector3 Link_get_translation(Link& self) { return self.translation(); }
void Link_set_translation(Link& self, const Vector3& p) { self.translation() = p; }
Matrix3 Link_get_rotation(Link& self) { return self.rotation(); }
void Link_set_rotation(Link& self, const Matrix3& R) { self.rotation() = R; }
Position Link_get_Tb(Link& self) { return self.Tb(); }
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
SgNodePtr Link_visualShape(const Link& self) { return self.visualShape(); }
SgNodePtr Link_collisionShape(const Link& self) { return self.collisionShape(); }
MappingPtr Link_info(Link& self) { return self.info(); }
py::object Link_info2(Link& self, const std::string& key, py::object defaultValue)
{
    if(!PyFloat_Check(defaultValue.ptr())){
        PyErr_SetString(PyExc_TypeError, "The argument type is not supported");
        py::throw_error_already_set();
    }
    double v = py::extract<double>(defaultValue);
    return py::object(self.info(key, v));
}
double Link_floatInfo(Link& self, const std::string& key) { return self.info<double>(key); }


BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(Body_calcForwardKinematics_overloads, calcForwardKinematics, 0, 2)

BodyPtr Body_clone(Body& self) { return self.clone(); }
LinkPtr Body_createLink1(Body& self) { return self.createLink(); }
LinkPtr Body_createLink2(Body& self, const Link* org) { return self.createLink(org); }
LinkPtr Body_joint(Body& self, int id) { return self.joint(id); }
LinkPtr Body_link1(Body& self, int index) { return self.link(index); }
LinkPtr Body_link2(Body& self, const std::string& name) { return self.link(name); }
LinkPtr Body_rootLink(Body& self) { return self.rootLink(); }
DevicePtr Body_device(Body& self, int index) { return self.device(index); }

PyObject* Body_calcTotalMomentum(Body& self) {
    Vector3 P, L;
    self.calcTotalMomentum(P, L);
    py::list p, l;
    for(int i=0; i < 3; ++i){
        p.append(P[i]);
        l.append(L[i]);
    }
    py::tuple ret = py::make_tuple(p, l);
    return py::incref(ret.ptr());
}


BodyPtr BodyLoader_load2(BodyLoader& self, const std::string& filename) { return self.load(filename); }

LinkPtr JointPath_joint(JointPath& self, int index) { return self.joint(index); }
LinkPtr JointPath_baseLink(JointPath& self) { return self.baseLink(); }
LinkPtr JointPath_endLink(JointPath& self) { return self.endLink(); }

bool (JointPath::*JointPath_calcInverseKinematics)(const Vector3& , const Matrix3&) = &JointPath::calcInverseKinematics;
bool (JointPath::*JointPath_calcInverseKinematics2)(const Vector3&, const Matrix3&, const Vector3&, const Matrix3&) = &JointPath::calcInverseKinematics;

MultiValueSeqPtr BodyMotion_get_jointPosSeq(BodyMotion& self) { return self.jointPosSeq(); }
void BodyMotion_set_jointPosSeq(BodyMotion& self, const MultiValueSeqPtr& jointPosSeq) { self.jointPosSeq() = jointPosSeq; }
MultiSE3SeqPtr BodyMotion_get_linkPosSeq(BodyMotion& self) { return self.linkPosSeq(); }
void BodyMotion_set_linkPosSeq(BodyMotion& self, const MultiSE3SeqPtr& linkPosSeq) { self.linkPosSeq() = linkPosSeq; }

BodyMotion::Frame (BodyMotion::*BodyMotion_frame)(int) = &BodyMotion::frame;

DevicePtr Device_clone(Device& self) { return self.clone(); }
void Device_clearState(Device& self) { return self.clearState(); }
LinkPtr Device_link(Device& self) { return self.link(); }
Position Device_get_T_local(Device& self) { return (Position) self.T_local(); }
void Device_set_T_local(Device& self, const Position& T_local) { self.T_local() = T_local.matrix(); }

} // namespace

namespace cnoid 
{

BOOST_PYTHON_MODULE(Body)
{
    py::import("cnoid.Util");

    {
        py::scope linkScope = 
            py::class_<Link, LinkPtr, py::bases<Referenced>>("Link")
            .def("index", &Link::index)
            .def("getIndex", &Link::index)
            .def("isValid", &Link::isValid)
            .def("parent", Link_parent)
            .def("getParent", Link_parent)
            .def("sibling", Link_sibling)
            .def("getSibling", Link_sibling)
            .def("child", Link_child)
            .def("getChild", Link_child)
            .def("isRoot", &Link::isRoot)
            .add_property("T", Link_get_position, Link_set_position)
            .def("position", Link_get_position)
            .def("getPosition", Link_get_position)
            .def("setPosition", Link_set_position)
            .add_property("p", Link_get_translation, Link_set_translation)
            .def("translation", Link_get_translation)
            .def("getTranslation", Link_get_translation)
            .def("setTranslation", Link_set_translation)
            .add_property("R", Link_get_rotation, Link_set_rotation)
            .def("rotation", Link_get_rotation)
            .def("getRotation", Link_get_rotation)
            .def("setRotation", Link_set_rotation)
            .add_property("Tb", Link_get_Tb)
            .def("b", Link_get_offsetTranslation)
            .def("offsetTranslation", Link_get_offsetTranslation)
            .def("getOffsetTranslation", Link_get_offsetTranslation)
            .add_property("Rb", Link_get_offsetRotation)
            .def("offsetRotation", Link_get_offsetRotation)
            .def("getOffsetRotation", Link_get_offsetRotation)
            .def("jointId", &Link::jointId)
            .def("getJointId", &Link::jointId)
            .def("jointType", &Link::jointType)
            .def("getJointType", &Link::jointType)
            .def("isFixedJoint", &Link::isFixedJoint)
            .def("isFreeJoint", &Link::isFreeJoint)
            .def("isRotationalJoint", &Link::isRotationalJoint)
            .def("isSlideJoint", &Link::isSlideJoint)
            .add_property("a", py::make_function(&Link::a, py::return_value_policy<py::return_by_value>()))
            .def("jointAxis", &Link::jointAxis, py::return_value_policy<py::return_by_value>())
            .def("getJointAxis", &Link::jointAxis, py::return_value_policy<py::return_by_value>())
            .add_property("d", py::make_function(&Link::d, py::return_value_policy<py::return_by_value>()))
            .add_property("q", Link_get_q, Link_set_q)
            .add_property("dq", Link_get_dq, Link_set_dq)
            .add_property("ddq", Link_get_ddq, Link_set_ddq)
            .add_property("u", Link_get_u, Link_set_u)
            .add_property("q_upper", &Link::q_upper)
            .add_property("q_lower", &Link::q_lower)
            .add_property("dq_upper", &Link::dq_upper)
            .add_property("dq_lower", &Link::dq_lower)
            .add_property("v", Link_get_v, Link_set_v)
            .add_property("w", Link_get_w, Link_set_w)
            .add_property("dv", Link_get_dv, Link_set_dv)
            .add_property("dw", Link_get_dw, Link_set_dw)
            .add_property("c", py::make_function(&Link::c, py::return_value_policy<py::return_by_value>()))
            .def("centerOfMass", &Link::centerOfMass, py::return_value_policy<py::return_by_value>())
            .def("getCenterOfMass", &Link::centerOfMass, py::return_value_policy<py::return_by_value>())
            .add_property("wc", py::make_function(Link_get_wc, py::return_value_policy<py::return_by_value>()), Link_set_wc)
            .def("centerOfMassGlobal", &Link::centerOfMassGlobal, py::return_value_policy<py::return_by_value>())
            .def("getCenterOfMassGlobal", &Link::centerOfMassGlobal, py::return_value_policy<py::return_by_value>())
            .add_property("m", &Link::m)
            .def("mass", &Link::mass)
            .def("getMass", &Link::mass)
            .add_property("I", py::make_function(&Link::I, py::return_value_policy<py::return_by_value>()))
            .add_property("Jm2", &Link::Jm2)
            .add_property("F_ext", Link_get_F_ext, Link_set_F_ext)
            .add_property("f_ext", Link_get_f_ext, Link_set_f_ext)
            .add_property("tau_ext", Link_get_tau_ext, Link_set_tau_ext)
            .def("name", &Link::name, py::return_value_policy<py::copy_const_reference>())
            .def("getName", &Link::name, py::return_value_policy<py::copy_const_reference>())
            .def("visualShape", Link_visualShape)
            .def("getVisualShape", Link_visualShape)
            .def("collisionShape", Link_collisionShape)
            .def("getCollisionShape", Link_collisionShape)
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
            .def("info", Link_info)
            .def("getInfo", Link_info)
            .def("info", Link_info2)
            .def("getInfo", Link_info2)
            .def("floatInfo", Link_floatInfo)
            .def("getFloatInfo", Link_floatInfo)
            ;

        py::enum_<Link::JointType>("JointType")
            .value("REVOLUTE_JOINT", Link::REVOLUTE_JOINT)
            .value("PRISMATIC_JOINT", Link::PRISMATIC_JOINT) 
            .value("FREE_JOINT", Link::FREE_JOINT) 
            .value("FIXED_JOINT", Link::FIXED_JOINT);
    }        
    
    {
        py::scope bodyScope =
            py::class_<Body, BodyPtr, py::bases<Referenced>>("Body")
            .def("clone", Body_clone)
            .def("createLink", Body_createLink1)
            .def("createLink", Body_createLink2)
            .def("name", &Body::name, py::return_value_policy<py::copy_const_reference>())
            .def("getName", &Body::name, py::return_value_policy<py::copy_const_reference>())
            .def("setName", &Body::setName)
            .def("modelName", &Body::modelName, py::return_value_policy<py::copy_const_reference>())
            .def("getModelName", &Body::modelName, py::return_value_policy<py::copy_const_reference>())
            .def("setModelName", &Body::setModelName)
            .def("setRootLink", &Body::setRootLink)
            .def("updateLinkTree", &Body::updateLinkTree)
            .def("initializeState", &Body::initializeState)
            .def("numJoints", &Body::numJoints)
            .def("getNumJoints", &Body::numJoints)
            .def("numVirtualJoints", &Body::numVirtualJoints)
            .def("getNumVirtualJoints", &Body::numVirtualJoints)
            .def("numAllJoints", &Body::numAllJoints)
            .def("getNumAllJoints", &Body::numAllJoints)
            .def("joint", Body_joint)
            .def("getJoint", Body_joint)
            .def("numLinks", &Body::numLinks)
            .def("getNumLinks", &Body::numLinks)
            .def("link", Body_link1)
            .def("getLink", Body_link1)
            .def("link", Body_link2)
            .def("getLink", Body_link2)
            .def("rootLink", Body_rootLink)
            .def("getRootLink", Body_rootLink)
            .def("numDevices", &Body::numDevices)
            .def("getNumDevices", &Body::numDevices)
            .def("device", Body_device)
            .def("getDevice", Body_device)
            .def("addDevice", &Body::addDevice)
            .def("initializeDeviceStates", &Body::initializeDeviceStates)
            .def("clearDevices", &Body::clearDevices)
            .def("isStaticModel", &Body::isStaticModel)
            .def("isFixedRootModel", &Body::isFixedRootModel)
            .def("resetDefaultPosition", &Body::resetDefaultPosition)
            .def("defaultPosition", &Body::defaultPosition, py::return_value_policy<py::return_by_value>())
            .def("getDefaultPosition", &Body::defaultPosition, py::return_value_policy<py::return_by_value>())
            .def("mass", &Body::mass)
            .def("getMass", &Body::mass)
            .def("calcCenterOfMass", &Body::calcCenterOfMass, py::return_value_policy<py::return_by_value>())
            .def("centerOfMass", &Body::centerOfMass, py::return_value_policy<py::return_by_value>())
            .def("getCenterOfMass", &Body::centerOfMass, py::return_value_policy<py::return_by_value>())
            .def("calcTotalMomentum", Body_calcTotalMomentum)
            .def("calcForwardKinematics", &Body::calcForwardKinematics, Body_calcForwardKinematics_overloads())
            .def("clearExternalForces", &Body::clearExternalForces)
            .def("numExtraJoints", &Body::numExtraJoints)
            .def("getNumExtraJoints", &Body::numExtraJoints)
            //.def("extraJoint", extraJoint, , return_value_policy<reference_existing_object>())
            //.def("addExtraJoint", &Body::addExtraJoint)
            .def("clearExtraJoints", &Body::clearExtraJoints)
            //.def("installCustomizer", installCustomizer)
            .def("hasVirtualJointForces", &Body::hasVirtualJointForces)
            .def("setVirtualJointForces", &Body::setVirtualJointForces)
            .def("addCustomizerDirectory", &Body::addCustomizerDirectory).staticmethod("addCustomizerDirectory")
            .def(py::other<BodyMotion::ConstFrame>() >> py::self)
            ;
    }

    py::implicitly_convertible<BodyPtr, ReferencedPtr>();

    {
        py::scope ExtraJointScope = py::class_<ExtraJoint>("ExtraJoint", py::init<ExtraJoint::ExtraJointType, const Vector3&>())
            .def("setType", &ExtraJoint::setType)
            .def("setAxis", &ExtraJoint::setAxis)
            .def("setPoint", &ExtraJoint::setPoint)
            ;

        py::enum_<ExtraJoint::ExtraJointType>("ExtraJointType")
              .value("EJ_PISTON", ExtraJoint::EJ_PISTON)
              .value("EJ_BALL", ExtraJoint::EJ_BALL);
    }

    py::class_<AbstractBodyLoader, boost::noncopyable>("AbstractBodyLoader", py::no_init)
        .def("setVerbose", &AbstractBodyLoader::setVerbose)
        .def("setShapeLoadingEnabled", &AbstractBodyLoader::setShapeLoadingEnabled)
        .def("setDefaultDivisionNumber", &AbstractBodyLoader::setDefaultDivisionNumber)
        .def("setDefaultCreaseAngle", &AbstractBodyLoader::setDefaultCreaseAngle)
        .def("load", &AbstractBodyLoader::load)
        ;

    py::class_<BodyLoader, py::bases<AbstractBodyLoader>>("BodyLoader")
        .def("load", BodyLoader_load2)
        .def("lastActualBodyLoader", &BodyLoader::lastActualBodyLoader)
        .def("getLastActualBodyLoader", &BodyLoader::lastActualBodyLoader)
        ;

    {
        py::scope jointPathScope =
            py::class_<JointPath, JointPathPtr, boost::noncopyable>("JointPath", py::init<>())
            .def("numJoints", &JointPath::numJoints)
            .def("getNumJoints", &JointPath::numJoints)
            .def("joint", JointPath_joint)
            .def("getJoint", JointPath_joint)
            .def("baseLink", JointPath_baseLink)
            .def("getBaseLink", JointPath_baseLink)
            .def("endLink", JointPath_endLink)
            .def("getEndLink", JointPath_endLink)
            .def("indexOf", &JointPath::indexOf)
            .def("getIndexOf", &JointPath::indexOf)
            .def("customizeTarget", &JointPath::customizeTarget)
            .def("numIterations", &JointPath::numIterations)
            .def("getNumIterations", &JointPath::numIterations)
            .def("calcJacobian", &JointPath::calcJacobian)
            .def("calcInverseKinematics", JointPath_calcInverseKinematics)
            .def("calcInverseKinematics", JointPath_calcInverseKinematics2)
            ;
    }

    py::def("getCustomJointPath", getCustomJointPath);

    {
        py::scope bodyMotionScope =
            py::class_<BodyMotion, BodyMotionPtr, py::bases<AbstractSeq>>("BodyMotion")
            .def("setNumJoints", &BodyMotion::setNumJoints)
            .def("numJoints", &BodyMotion::numJoints)
            .def("getNumJoints", &BodyMotion::numJoints)
            .def("numLinks", &BodyMotion::numLinks)
            .def("getNumLinks", &BodyMotion::numLinks)
            .def("frameRate", &BodyMotion::frameRate)
            .def("getFrameRate", &BodyMotion::frameRate)
            .def("getFrameRate",&BodyMotion::getFrameRate)
            .def("setFrameRate", &BodyMotion::setFrameRate)
            .def("getOffsetTimeFrame", &BodyMotion::getOffsetTimeFrame)
            .def("numFrames", &BodyMotion::numFrames)
            .def("getNumFrames", &BodyMotion::getNumFrames)
            .def("setNumFrames", &BodyMotion::setNumFrames)
            .def("getJointPosSeq", BodyMotion_get_jointPosSeq)
            .def("getLinkPosSeq", BodyMotion_get_linkPosSeq)
            .add_property("jointPosSeq", BodyMotion_get_jointPosSeq, BodyMotion_set_jointPosSeq)
            .add_property("linkPosSeq", BodyMotion_get_linkPosSeq, BodyMotion_set_linkPosSeq)
            .def("frame", BodyMotion_frame)
            .def("getFrame", BodyMotion_frame)
            // deprecated
            .def("setNumParts", &BodyMotion::setNumJoints)
            .def("getNumParts", &BodyMotion::numJoints)
            ;

        py::class_<BodyMotion::Frame>("Frame", py::no_init)
            .def("frame", &BodyMotion::Frame::frame)
            .def("getFrame", &BodyMotion::Frame::frame)
            .def(py::self << py::other<Body>())
            .def(py::other<Body>() >> py::self)
            ;
    }

    py::implicitly_convertible<BodyMotionPtr, AbstractSeqPtr>();

    {
        py::scope deviceScope =
            py::class_<Device, DevicePtr, boost::noncopyable>("Device", py::no_init)
            .def("setIndex", &Device::setIndex)
            .def("setId", &Device::setId)
            .def("setName", &Device::setName)
            .def("setLink", &Device::setLink)
            .def("clone", &Device_clone)
            .def("clearState", &Device_clearState)
            .def("hasStateOnly", &Device::hasStateOnly)
            .def("index", &Device::index)
            .def("getIndex", &Device::index)
            .def("id", &Device::id)
            .def("getId", &Device::id)
            .def("name", &Device::name, py::return_value_policy<py::copy_const_reference>())
            .def("getName", &Device::name, py::return_value_policy<py::copy_const_reference>())
            .def("link", &Device_link)
            .def("getLink", &Device_link)
            .add_property("T_local", Device_get_T_local, Device_set_T_local)
            ;
    }

#ifdef _MSC_VER
    py::register_ptr_to_python<BodyPtr>();
    py::register_ptr_to_python<LinkPtr>();
#endif

}

}; // namespace cnoid
