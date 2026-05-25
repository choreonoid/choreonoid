#include "../Link.h"
#include "../Body.h"
#include <cnoid/ValueTree>
#include <cnoid/SceneGraph>
#include <cnoid/SceneDrawables>
#include <cnoid/PyUtil>
#include <cnoid/PyEigenTypes>
#include <cnoid/PyNumpyHelper>
#include <nanobind/stl/vector.h>

using namespace std;
using namespace cnoid;
namespace nb = nanobind;

namespace {

nb::object Link_getInfo(Link& self, const std::string& key, nb::object defaultValue)
{
    if(!PyFloat_Check(defaultValue.ptr())){
        PyErr_SetString(PyExc_TypeError, "The argument type is not supported");
        throw nb::python_error();
    }
    double v = nb::cast<double>(defaultValue);
    return nb::cast(self.info(key, v));
}

}

namespace cnoid {

void exportPyLink(nb::module_& m)
{
    nb::class_<Link, Referenced> link(m, "Link");
    link
        .def("__repr__", [](const Link& self){ return "<cnoid.Body.Link named '" + self.name() + "'>"; })
        .def_prop_ro("name", &Link::name)
        .def_prop_ro("index", &Link::index)
        .def("isValid", &Link::isValid)
        .def("isRoot", &Link::isRoot)
        .def("isStatic", &Link::isStatic)
        .def("isFixedToRoot", &Link::isFixedToRoot)
        .def("isOwnerOf", &Link::isOwnerOf)
        .def("isEndLink", &Link::isEndLink)
        .def_prop_ro("body", (Body*(Link::*)()) &Link::body)
        .def_prop_ro("parent", &Link::parent)
        .def_prop_ro("sibling", &Link::sibling)
        .def_prop_ro("child", &Link::child)
        .def_prop_rw(
            "T",
            [](Link& self) -> Isometry3::MatrixType& { return self.T().matrix(); },
            [](Link& self, const python::Matrix4RMArg& T){ self.setPosition(T.value); })
        .def_prop_rw(
            "position",
            [](Link& self) -> Isometry3::MatrixType& { return self.position().matrix(); },
            [](Link& self, const python::Matrix4RMArg& T){ self.setPosition(T.value); })
        .def("getPosition", [](Link& self) -> Isometry3::MatrixType& { return self.T().matrix(); })
        .def("setPosition", [](Link& self, const python::Matrix4RMArg& T){ self.setPosition(T.value); })
        .def_prop_rw(
            "p",
            [](Link& self){ return python::eigenVectorView(self.p(), nb::find(&self)); },
            [](Link& self, const python::Vector3Arg& p){ self.p() = p.value; })
        .def_prop_rw(
            "translation",
            [](Link& self){ return python::eigenVectorView(self.translation(), nb::find(&self)); },
            [](Link& self, const python::Vector3Arg& p){ self.setTranslation(p.value); })
        .def("setTranslation", [](Link& self, const python::Vector3Arg& p){ self.setTranslation(p.value); })
        .def_prop_rw(
            "R",
            [](Link& self){ return python::eigenMatrixView(self.R(), nb::find(&self)); },
            [](Link& self, const python::Matrix3RMArg& R){ self.R() = R.value; })
        .def_prop_rw(
            "rotation",
            [](Link& self){ return python::eigenMatrixView(self.rotation(), nb::find(&self)); },
            [](Link& self, const python::Matrix3RMArg& R){ self.rotation() = R.value; })
        .def("setRotation", [](Link& self, const python::Matrix3RMArg& R){ self.setRotation(R.value); })
        .def("setRotation", [](Link& self, const AngleAxis& aa){ self.setRotation(aa); })
        .def_prop_ro("Tb", [](const Link& self) -> Isometry3::ConstMatrixType& { return self.Tb().matrix(); })
        .def_prop_ro("b", [](const Link& self){ return self.b(); })
        .def_prop_ro("offsetTranslation", [](const Link& self){ return self.offsetTranslation(); })
        .def_prop_ro("Rb", [](const Link& self){ return self.Rb(); })
        .def_prop_ro("offsetRotation", [](const Link& self){ return self.offsetRotation(); })
        .def_prop_ro("jointId", &Link::jointId)
        .def_prop_ro("jointName", &Link::jointName)
        .def_prop_ro("jointSpecificName", &Link::jointSpecificName)
        .def_prop_ro("jointType", &Link::jointType)
        .def_prop_ro("jointTypeLabel", [](const Link& self){ return self.jointTypeLabel(); })
        .def_prop_ro("jointTypeSymbol", [](const Link& self){ return self.jointTypeSymbol(); })
        .def("isFixedJoint", &Link::isFixedJoint)
        .def("isFreeJoint", &Link::isFreeJoint)
        .def("isRevoluteJoint", &Link::isRevoluteJoint)
        .def("isPrismaticJoint", &Link::isPrismaticJoint)
        .def("hasActualJoint", &Link::hasActualJoint)
        .def_prop_ro("a", &Link::a)
        .def_prop_ro("jointAxis", &Link::jointAxis)
        .def_prop_ro("d", &Link::d)
        .def_prop_ro("Jm2", &Link::Jm2)
        .def_prop_rw("actuationMode", &Link::actuationMode, &Link::setActuationMode)
        .def("setActuationMode", &Link::setActuationMode)
        .def_prop_rw("sensingMode", &Link::sensingMode, &Link::setSensingMode)
        .def("setSensingMode", &Link::setSensingMode)
        .def("mergeSensingMode", &Link::mergeSensingMode)
        .def("getStateModeString", &Link::getStateModeString)
        .def_prop_rw("q", (double(Link::*)()const)&Link::q, [](Link& self, double q){ self.q() = q; })
        .def_prop_rw("dq", (double(Link::*)()const)&Link::dq, [](Link& self, double dq){ self.dq() = dq; })
        .def_prop_rw("ddq", (double(Link::*)()const)&Link::ddq, [](Link& self, double ddq){ self.ddq() = ddq; })
        .def_prop_rw("u", (double(Link::*)()const)&Link::u, [](Link& self, double u){ self.u() = u; })
        .def_prop_rw("q_target", (double(Link::*)()const)&Link::q_target, [](Link& self, double q){ self.q_target() = q; })
        .def_prop_rw("dq_target", (double(Link::*)()const)&Link::dq_target, [](Link& self, double dq){ self.dq_target() = dq; })
        .def_prop_ro("q_initial", (double(Link::*)()const)&Link::q_initial)
        .def_prop_ro("q_upper", (double(Link::*)()const)&Link::q_upper)
        .def_prop_ro("q_lower", (double(Link::*)()const)&Link::q_lower)
        .def_prop_ro("dq_upper", (double(Link::*)()const)&Link::dq_upper)
        .def_prop_ro("dq_lower", (double(Link::*)()const)&Link::dq_lower)
        .def_prop_ro("u_upper", (double(Link::*)()const)&Link::u_upper)
        .def_prop_ro("u_lower", (double(Link::*)()const)&Link::u_lower)
        .def("hasJointDisplacementLimits", &Link::hasJointDisplacementLimits)
        .def("hasJointVelocityLimits", &Link::hasJointVelocityLimits)
        .def("hasJointEffortLimits", &Link::hasJointEffortLimits)
        .def("setUnlimitedJointDisplacementRange", &Link::setUnlimitedJointDisplacementRange)
        .def("setUnlimitedJointVelocityRange", &Link::setUnlimitedJointVelocityRange)
        .def("setUnlimitedEffortRange", &Link::setUnlimitedEffortRange)
        .def_prop_rw(
            "v",
            [](Link& self){ return python::eigenVectorView(self.v(), nb::find(&self)); },
            [](Link& self, const python::Vector3Arg& v){ self.v() = v.value; })
        .def_prop_rw(
            "w",
            [](Link& self){ return python::eigenVectorView(self.w(), nb::find(&self)); },
            [](Link& self, const python::Vector3Arg& w){ self.w() = w.value; })
        .def_prop_rw(
            "dv",
            [](Link& self){ return python::eigenVectorView(self.dv(), nb::find(&self)); },
            [](Link& self, const python::Vector3Arg& dv){ self.dv() = dv.value; })
        .def_prop_rw(
            "dw",
            [](Link& self){ return python::eigenVectorView(self.dw(), nb::find(&self)); },
            [](Link& self, const python::Vector3Arg& dw){ self.dw() = dw.value; })
        .def_prop_ro("c", &Link::c)
        .def_prop_ro("centerOfMass", &Link::centerOfMass)
        .def_prop_rw(
            "wc",
            [](Link& self){ return python::eigenVectorView(self.wc(), nb::find(&self)); },
            [](Link& self, const python::Vector3Arg& wc){ self.wc() = wc.value; })
        .def_prop_ro("centerOfMassGlobal", [](Link& self){ return python::eigenVectorView(self.centerOfMassGlobal(), nb::find(&self)); })
        .def_prop_ro("m", &Link::m)
        .def_prop_ro("mass", &Link::mass)
        .def_prop_ro("I", [](Link& self){ return python::eigenMatrixView(self.I(), nb::find(&self)); })
        .def_prop_rw(
            "externalWrench",
            [](Link& self){ return python::eigenVectorView(self.externalWrench(), nb::find(&self)); },
            [](Link& self, const python::Vector6Arg& F){ self.externalWrench() = F.value; })
        .def_prop_rw(
            "externalForce",
            [](Link& self){ return python::eigenVectorView(self.externalWrench().topRows<3>(), nb::find(&self)); },
            [](Link& self, const python::Vector3Arg& f){ self.externalForce() = f.value; })
        .def_prop_rw(
            "externalTorque",
            [](Link& self){ return python::eigenVectorView(self.externalWrench().bottomRows<3>(), nb::find(&self)); },
            [](Link& self, const python::Vector3Arg& f){ self.externalTorque() = f.value; })
        .def_prop_rw(
            "F_ext",
            [](Link& self){ return python::eigenVectorView(self.F_ext(), nb::find(&self)); },
            [](Link& self, const python::Vector6Arg& F){ self.F_ext() = F.value; })
        .def_prop_rw(
            "f_ext",
            [](Link& self){ return python::eigenVectorView(self.F_ext().topRows<3>(), nb::find(&self)); },
            [](Link& self, const python::Vector3Arg& f){ self.f_ext() = f.value; })
        .def_prop_rw(
            "tau_ext",
            [](Link& self){ return python::eigenVectorView(self.F_ext().bottomRows<3>(), nb::find(&self)); },
            [](Link& self, const python::Vector3Arg& tau_ext){ self.tau_ext() = tau_ext.value; })
        .def("addExternalForceAtLocalPosition",
             [](Link& self, const python::Vector3Arg& f, const python::Vector3Arg& p){
                 self.addExternalForceAtLocalPosition(f.value, p.value); })
        .def("addExternalForceAtGlobalPosition",
             [](Link& self, const python::Vector3Arg& f, const python::Vector3Arg& p){
                 self.addExternalForceAtGlobalPosition(f.value, p.value); })
        .def_prop_ro("materialId", &Link::materialId)
        .def_prop_ro("materialName", &Link::materialName)
        .def_prop_ro("contactPoints", [](const Link& self){ return self.contactPoints(); })
        .def_prop_ro("shape", &Link::shape)
        .def_prop_ro("visualShape", &Link::visualShape)
        .def_prop_ro("collisionShape", &Link::collisionShape)
        .def_prop_ro("hasDedicatedCollisionShape", &Link::hasDedicatedCollisionShape)
        .def("setParent", &Link::setParent)
        .def("setIndex", &Link::setIndex)
        .def("setName", &Link::setName)
        .def("prependChild", &Link::prependChild)
        .def("appendChild", &Link::appendChild)
        .def("removeChild", &Link::removeChild)
        .def("setOffsetPosition",
             [](Link& self, const python::Matrix4RMArg& T){ self.setOffsetPosition(Isometry3(T.value)); })
        .def("setOffsetTranslation", [](Link& self, const python::Vector3Arg& p){ self.setOffsetTranslation(p.value); })
        .def("setOffsetRotation", [](Link& self, const python::Matrix3RMArg& R){ self.setOffsetRotation(R.value); })
        .def("setOffsetRotation", [](Link& self, const AngleAxis& aa){ self.setOffsetRotation(aa); })
        .def("setJointType", &Link::setJointType)
        .def("setJointId", &Link::setJointId)
        .def("setJointName", &Link::setJointName)
        .def("resetJointSpecificName", &Link::resetJointSpecificName)
        .def("setJointAxis", [](Link& self, const python::Vector3Arg& axis){ self.setJointAxis(axis.value); })
        .def("setInitialJointAngle", &Link::setInitialJointAngle)
        .def("setInitialJointDisplacement", &Link::setInitialJointDisplacement)
        .def("setJointRange", &Link::setJointRange)
        .def("setJointVelocityRange", &Link::setJointVelocityRange)
        .def("setJointEffortRange", &Link::setJointEffortRange)
        .def("setMass", &Link::setMass)
        .def("setInertia", [](Link& self, const python::Matrix3Arg& I){ self.setInertia(I.value); })
        .def("setCenterOfMass", [](Link& self, const python::Vector3Arg& c){ self.setCenterOfMass(c.value); })
        .def("setEquivalentRotorInertia", &Link::setEquivalentRotorInertia)
        .def("setMaterial", (void(Link::*)(int)) &Link::setMaterial)
        .def("setMaterial", (void(Link::*)(const std::string&)) &Link::setMaterial)
        .def("addShapeNode", [](Link& self, SgNode* shape){ self.addShapeNode(shape, true); })
        .def("addVisualShapeNode", [](Link& self, SgNode* shape){ self.addVisualShapeNode(shape, true); })
        .def("addCollisionShapeNode", [](Link& self, SgNode* shape){ self.addCollisionShapeNode(shape, true); })
        .def("removeShapeNode", [](Link& self, SgNode* shape){ self.removeShapeNode(shape, true); })
        .def("clearShapeNodes", [](Link& self){ self.clearShapeNodes(true); })
        .def_prop_ro("info", (Mapping*(Link::*)()) &Link::info)
        .def("getInfo", Link_getInfo)
        .def("floatInfo", [](Link& self, const std::string& key){ return self.info<double>(key); })
        ;

    nb::enum_<Link::JointType>(link, "JointType", nb::is_arithmetic())
        .value("RevoluteJoint", Link::RevoluteJoint)
        .value("PrismaticJoint", Link::PrismaticJoint)
        .value("FreeJoint", Link::FreeJoint)
        .value("FixedJoint", Link::FixedJoint)
        .value("PseudoContinuousTrackJoint", Link::PseudoContinuousTrackJoint)
        .export_values();

    nb::enum_<Link::StateFlag>(link, "StateFlag", nb::is_arithmetic())
        .value("StateNone", Link::StateNone)
        .value("JointDisplacement", Link::JointDisplacement)
        .value("JointAngle", Link::JointAngle)
        .value("JointVelocity", Link::JointVelocity)
        .value("JointAcceleration", Link::JointAcceleration)
        .value("JointEffort", Link::JointEffort)
        .value("JointForce", Link::JointForce)
        .value("JointTorque", Link::JointTorque)
        .value("LinkPosition", Link::LinkPosition)
        .value("LinkTwist", Link::LinkTwist)
        .value("LinkExtWrench", Link::LinkExtWrench)
        .value("LinkContactState", Link::LinkContactState)
        .export_values();

    nb::class_<Link::ContactPoint>(link, "ContactPoint")
        .def_prop_ro("position", &Link::ContactPoint::position)
        .def_prop_ro("normal", &Link::ContactPoint::normal)
        .def_prop_ro("force", &Link::ContactPoint::force)
        .def_prop_ro("velocity", &Link::ContactPoint::velocity)
        .def_prop_ro("depth", &Link::ContactPoint::depth)
        ;
}

}
