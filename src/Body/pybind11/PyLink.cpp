/**
  @author Shin'ichiro Nakaoka
*/

#include "../Link.h"
#include "../Body.h"
#include <cnoid/ValueTree>
#include <cnoid/SceneGraph>
#include <cnoid/SceneDrawables>
#include <cnoid/PyUtil>

using namespace std;
using namespace cnoid;
namespace py = pybind11;

namespace {

using Matrix4RM = Eigen::Matrix<double, 4, 4, Eigen::RowMajor>;
using Matrix3RM = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>;

py::object Link_info2(Link& self, const std::string& key, py::object defaultValue)
{
    if(!PyFloat_Check(defaultValue.ptr())){
        PyErr_SetString(PyExc_TypeError, "The argument type is not supported");
        throw py::error_already_set();
    }
    double v = defaultValue.cast<double>();
    return py::cast(self.info(key, v));
}

}

namespace cnoid {

void exportPyLink(py::module& m)
{
    py::class_<Link, LinkPtr, Referenced> link(m, "Link");
    link
        .def("__repr__", [](const Link &self) {return "<cnoid.Body.Link named '" + self.name() + "'>"; })
        .def_property_readonly("name", &Link::name)
        .def_property_readonly("index", &Link::index)
        .def("isValid", &Link::isValid)
        .def("isRoot", &Link::isRoot)
        .def("isStatic", &Link::isStatic)
        .def("isFixedToRoot", &Link::isFixedToRoot)
        .def("isOwnerOf", &Link::isOwnerOf)
        .def_property_readonly("body", (Body*(Link::*)())&Link::body)
        .def_property_readonly("parent", &Link::parent)
        .def_property_readonly("sibling", &Link::sibling)
        .def_property_readonly("child", &Link::child)
        .def_property(
            "T",
            [](Link& self) -> Isometry3::MatrixType& { return self.T().matrix(); },
            [](Link& self, Eigen::Ref<const Matrix4RM> T){ self.setPosition(T); })
        .def_property(
            "position",
            [](Link& self) -> Isometry3::MatrixType& { return self.position().matrix(); },
            [](Link& self, Eigen::Ref<const Matrix4RM> T){ self.setPosition(T); })
        .def("getPosition",
             [](const Link& self) { return self.T().matrix(); })
        .def("setPosition", [](Link& self, Eigen::Ref<const Matrix4RM> T){ self.setPosition(T); })
        .def_property(
            "p",
            [](Link& self){ return self.p(); },
            [](Link& self, Eigen::Ref<const Vector3> p){ self.p() = p; })
        .def_property(
            "translation",
            [](Link& self){ return self.translation(); },
            [](Link& self, Eigen::Ref<const Vector3> p){ self.setTranslation(p); })
        .def("setTranslation", [](Link& self, Eigen::Ref<const Vector3> p){ self.setTranslation(p); })
        .def_property(
            "R",
            [](Link& self){ return self.R(); },
            [](Link& self, Eigen::Ref<const Matrix3RM> R){ self.R() = R; })
        .def_property(
            "rotation",
            [](Link& self){ return self.rotation(); },
            [](Link& self, Eigen::Ref<const Matrix3RM> R){ self.rotation() = R; })
        .def("setRotation", [](Link& self, Eigen::Ref<const Matrix3RM> R){ self.setRotation(R); })
        .def("setRotation", [](Link& self, const AngleAxis& aa){ self.setRotation(aa); })
        .def_property_readonly(
            "Tb", [](const Link& self) -> Isometry3::ConstMatrixType& { return self.Tb().matrix(); })
        .def_property_readonly("b", [](const Link& self){ return self.b(); })
        .def_property_readonly("offsetTranslation", [](const Link& self){ return self.offsetTranslation(); })
        .def_property_readonly("Rb", [](const Link& self){ return self.Rb(); })
        .def_property_readonly("offsetRotation", [](const Link& self){ return self.offsetRotation(); })
        .def_property_readonly("jointId", &Link::jointId)
        .def_property_readonly("jointType", &Link::jointType)
        .def_property_readonly("jointTypeLabel", [](const Link& self){ return self.jointTypeLabel(); })
        .def_property_readonly("jointTypeSymbol", [](const Link& self){ return self.jointTypeSymbol(); })
        .def("isFixedJoint", &Link::isFixedJoint)
        .def("isFreeJoint", &Link::isFreeJoint)
        .def("isRevoluteJoint", &Link::isRevoluteJoint)
        .def("isPrismaticJoint", &Link::isPrismaticJoint)
        .def_property_readonly("a", &Link::a)
        .def_property_readonly("jointAxis", &Link::jointAxis)
        .def_property_readonly("d", &Link::d)
        .def_property_readonly("Jm2", &Link::Jm2)
        .def_property("actuationMode", &Link::actuationMode, &Link::setActuationMode)
        .def("setActuationMode", &Link::setActuationMode)
        .def_property("sensingMode", &Link::sensingMode, &Link::setSensingMode)
        .def("setSensingMode", &Link::setSensingMode)
        .def("mergeSensingMode", &Link::mergeSensingMode)
        .def("getStateModeString", &Link::getStateModeString)
        .def_property("q", (double&(Link::*)())&Link::q, [](Link& self, double q){ self.q() = q; })
        .def_property("dq", (double&(Link::*)())&Link::dq, [](Link& self, double dq){ self.dq() = dq; })
        .def_property("ddq", (double&(Link::*)())&Link::ddq, [](Link& self, double ddq){ self.ddq() = ddq; })
        .def_property("u", (double&(Link::*)())&Link::u, [](Link& self, double u){ self.u() = u; })
        .def_property_readonly("q_upper", (double(Link::*)()const)&Link::q_upper)
        .def_property_readonly("q_lower", (double(Link::*)()const)&Link::q_lower)
        .def_property_readonly("dq_upper", (double(Link::*)()const)&Link::dq_upper)
        .def_property_readonly("dq_lower", (double(Link::*)()const)&Link::dq_lower)
        .def_property(
            "v",
            [](Link& self) -> Vector3& { return self.v(); },
            [](Link& self, Eigen::Ref<const Vector3> v){ self.v() = v; })
        .def_property(
            "w",
            [](Link& self) -> Vector3& { return self.w(); },
            [](Link& self, Eigen::Ref<const Vector3> w){ self.w() = w; })
        .def_property(
            "dv",
            [](Link& self) -> Vector3& { return self.dv(); },
            [](Link& self, Eigen::Ref<const Vector3> dv){ self.dv() = dv; })
        .def_property(
            "dw",
            [](Link& self) -> Vector3& { return self.dw(); },
            [](Link& self, Eigen::Ref<const Vector3> dw){ self.dw() = dw; })
        .def_property_readonly("c", &Link::c)
        .def_property_readonly("centerOfMass", &Link::centerOfMass)
        .def_property("wc", (Vector3&(Link::*)())&Link::wc, [](Link& self, const Vector3& wc){ self.wc() = wc; })
        .def_property_readonly("centerOfMassGlobal", (Vector3&(Link::*)())&Link::centerOfMassGlobal)
        .def_property_readonly("m", &Link::m)
        .def_property_readonly("mass", &Link::mass)
        .def_property_readonly("I", [](const Link& self) -> const Matrix3& { return self.I(); })
        .def_property(
            "externalWrench",
            (Vector6&(Link::*)())&Link::externalWrench,
            [](Link& self, const Vector6& F){ self.externalWrench() = F; })
        .def_property(
            "externalForce",
            // Use the topRows function here to allow direct writing
            [](Link& self) { return self.externalWrench().topRows<3>(); },
            [](Link& self, const Vector3& f){ self.externalForce() = f; })
        .def_property(
            "externalTorque",
            // Use the bottomRows function here to allow direct writing
            [](Link& self) { return self.externalWrench().bottomRows<3>(); },
            [](Link& self, const Vector3& f){ self.externalTorque() = f; })
        .def_property(
            "F_ext",
            (Vector6&(Link::*)())&Link::F_ext,
            [](Link& self, const Vector6& F){ self.F_ext() = F; })
        .def_property(
            "f_ext",
            [](Link& self) { return self.F_ext().topRows<3>(); },
            [](Link& self, const Vector3& f){ self.f_ext() = f; })
        .def_property(
            "tau_ext",
            [](Link& self) { return self.F_ext().bottomRows<3>(); },
            [](Link& self, const Vector3& tau_ext){ self.tau_ext() = tau_ext; })
        .def("addExternalForceAtLocalPosition", &Link::addExternalForceAtLocalPosition)
        .def("addExternalForceAtGlobalPosition", &Link::addExternalForceAtGlobalPosition)
        .def_property_readonly("materialId", &Link::materialId)
        .def_property_readonly("materialName", &Link::materialName)
        .def_property_readonly("contactPoints", [](const Link& self){ return self.contactPoints(); })
        .def_property_readonly("shape", &Link::shape)
        .def_property_readonly("visualShape", &Link::visualShape)
        .def_property_readonly("collisionShape", &Link::collisionShape)
        .def_property_readonly("hasDedicatedCollisionShape", &Link::hasDedicatedCollisionShape)
        .def("setIndex", &Link::setIndex)
        .def("setName", &Link::setName)
        .def("prependChild", &Link::prependChild)
        .def("appendChild", &Link::appendChild)
        .def("removeChild", &Link::removeChild)
        .def("setOffsetPosition",
             [](Link& self, Eigen::Ref<const Matrix4RM> T){ self.setOffsetPosition(Isometry3(T)); })
        .def("setOffsetTranslation", [](Link& self, Eigen::Ref<const Vector3> p){ self.setOffsetTranslation(p); })
        .def("setOffsetRotation", [](Link& self, Eigen::Ref<const Matrix3RM> R){ self.setOffsetRotation(R); })
        .def("setOffsetRotation", [](Link& self, const AngleAxis& aa){ self.setOffsetRotation(aa); })
        .def("setJointType", &Link::setJointType)
        .def("setJointId", &Link::setJointId)
        .def("setJointAxis", &Link::setJointAxis)
        .def("setJointRange", &Link::setJointRange)
        .def("setJointVelocityRange", &Link::setJointVelocityRange)
        .def("setMass", &Link::setMass)
        .def("setInertia", &Link::setInertia)
        .def("setCenterOfMass", &Link::setCenterOfMass)
        .def("setEquivalentRotorInertia", &Link::setEquivalentRotorInertia)
        .def("setMaterial", (void(Link::*)(int)) &Link::setMaterial)
        .def("setMaterial", (void(Link::*)(const std::string&)) &Link::setMaterial)
        .def("addShapeNode", [](Link& self, SgNode* shape){ self.addShapeNode(shape, true); })
        .def("addVisualShapeNode", [](Link& self, SgNode* shape){ self.addVisualShapeNode(shape, true); })
        .def("addCollisionShapeNode", [](Link& self, SgNode* shape){ self.addCollisionShapeNode(shape, true); })
        .def("removeShapeNode", [](Link& self, SgNode* shape){ self.removeShapeNode(shape, true); })
        .def("clearShapeNodes", [](Link& self){ self.clearShapeNodes(true); })
        .def("info", (Mapping*(Link::*)())&Link::info)
        .def("info", Link_info2)
        .def("floatInfo", [](Link& self, const std::string& key) { return self.info<double>(key); })

        // deprecated
        .def_property_readonly("jointTypeString", [](const Link& self){ return self.jointTypeLabel(); })
        .def("isRotationalJoint", &Link::isRotationalJoint)
        .def("isSlideJoint", &Link::isSlideJoint)
        .def_property(
            "attitude",
            [](Link& self){ return self.rotation(); },
            [](Link& self, Eigen::Ref<const Matrix3RM> R){ self.rotation() = R; })
        .def("setAttitude", [](Link& self, Eigen::Ref<const Matrix3RM> R){ self.setRotation(R); })
        .def("calcRfromAttitude", [](Link& self, Eigen::Ref<const Matrix3> Ra){ return Ra; })
        .def("addExternalForce", &Link::addExternalForceAtLocalPosition)
        .def("getName", &Link::name)
        .def("getIndex", &Link::index)
        .def("getParent", &Link::parent)
        .def("getSibling", &Link::sibling)
        .def("getChild", &Link::child)
        .def("getTranslation", [](Link& self){ return self.translation(); })
        .def("getRotation", [](Link& self){ return self.rotation(); })
        .def("getOffsetTranslation", [](const Link& self){ return self.offsetTranslation(); })
        .def("getOffsetRotation", [](const Link& self){ return self.offsetRotation(); })
        .def("getJointId", &Link::jointId)
        .def("getJointType", &Link::jointType)
        .def("getJointAxis", &Link::jointAxis)
        .def("getCenterOfMass", &Link::centerOfMass)
        .def("getCenterOfMassGlobal", (Vector3&(Link::*)())&Link::centerOfMassGlobal)
        .def("getMass", &Link::mass)
        .def("getMaterialId", &Link::materialId)
        .def("getMaterialName", &Link::materialName)
        .def("getShape", &Link::shape)
        .def("getVisualShape", &Link::visualShape)
        .def("getCollisionShape", &Link::collisionShape)
        .def("getAttitude", [](const Link& self){ return self.rotation(); })
        .def("getInfo", (Mapping*(Link::*)())&Link::info)
        .def("getInfo", Link_info2)
        .def("getFloatInfo", [](Link& self, const std::string& key) { return self.info<double>(key); })
        ;

    py::enum_<Link::JointType>(link, "JointType")
        .value("RevoluteJoint", Link::RevoluteJoint)
        .value("PrismaticJoint", Link::PrismaticJoint)
        .value("FreeJoint", Link::FreeJoint)
        .value("FixedJoint", Link::FixedJoint)
        .value("PseudoContinuousTrackJoint", Link::PseudoContinuousTrackJoint)
        // deprecated
        .value("ROTATIONAL_JOINT", Link::JointType::ROTATIONAL_JOINT)
        .value("SLIDE_JOINT", Link::JointType::SLIDE_JOINT)
        .value("FREE_JOINT", Link::JointType::FREE_JOINT)
        .value("FIXED_JOINT", Link::JointType::FIXED_JOINT)
        .export_values();
    
    py::enum_<Link::StateFlag>(link, "StateFlag")
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

    py::class_<Link::ContactPoint>(link, "ContactPoint")
        .def_property_readonly("position", &Link::ContactPoint::position)
        .def_property_readonly("normal", &Link::ContactPoint::normal)
        .def_property_readonly("force", &Link::ContactPoint::force)
        .def_property_readonly("velocity", &Link::ContactPoint::velocity)
        .def_property_readonly("depth", &Link::ContactPoint::depth)
        ;
}

}
