#include "../Body.h"
#include "../BodyMotion.h"
#include "PyDeviceList.h"
#include <cnoid/ValueTree>
#include <cnoid/CloneMap>
#include <cnoid/PyUtil>
#include <cnoid/PyEigenTypes>
#include <nanobind/stl/vector.h>

using namespace std;
using namespace cnoid;
namespace nb = nanobind;

namespace {

nb::object Body_getInfo(Body& self, const std::string& key, nb::object defaultValue)
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

void exportPyBody(nb::module_& m)
{
    nb::class_<Body, Referenced> body(m, "Body");
    body
        .def(nb::init<>())
        .def(nb::init<const std::string&>())
        .def("__repr__", [](const Body& self){ return "<cnoid.Body.Body named '" + self.name() + "'>"; })
        .def("clone", (Body*(Body::*)()const) &Body::clone)
        .def("createLink", &Body::createLink, nb::arg("org") = nullptr, nb::arg("cloneMap") = nullptr)
        .def_prop_rw("name", &Body::name, &Body::setName)
        .def("setName", &Body::setName)
        .def_prop_rw("modelName", &Body::modelName, &Body::setModelName)
        .def("setModelName", &Body::setModelName)
        .def("setRootLink", &Body::setRootLink)
        .def("updateLinkTree", &Body::updateLinkTree)
        .def("initializePosition", &Body::initializePosition)
        .def("initializeState", &Body::initializeState)
        .def_prop_ro("parentBody", &Body::parentBody)
        .def_prop_ro("parentBodyLink", &Body::parentBodyLink)
        .def("setParent", &Body::setParent)
        .def("resetParent", &Body::resetParent)
        .def("syncPositionWithParentBody",
             &Body::syncPositionWithParentBody, nb::arg("doForwardKinematics") = true)
        .def_prop_ro("numLinks", &Body::numLinks)
        .def("link", (Link*(Body::*)(int)const) &Body::link)
        .def("link", (Link*(Body::*)(const string&)const) &Body::link)
        .def("linkTraverse", &Body::linkTraverse)
        .def_prop_ro("links", &Body::links)
        .def_prop_ro("rootLink", &Body::rootLink)
        .def("findUniqueEndLink", &Body::findUniqueEndLink)
        .def_prop_ro("lastSerialLink", &Body::lastSerialLink)
        .def_prop_ro("numJoints", &Body::numJoints)
        .def_prop_ro("numVirtualJoints", &Body::numVirtualJoints)
        .def_prop_ro("numAllJoints", &Body::numAllJoints)
        .def("joint", (Link*(Body::*)(int)const) &Body::joint)
        .def("joint", (Link*(Body::*)(const string&)const) &Body::joint)
        .def_prop_ro("joints", [](Body& self){
             auto joints = self.allJoints();
             joints.resize(self.numJoints());
             return joints; })
        .def_prop_ro("allJoints", &Body::allJoints)
        .def_prop_ro("numDevices", &Body::numDevices)
        .def_prop_ro("devices", [](Body& self){ return self.devices(); })
        .def("device", &Body::device)
        .def("findDevice", [](Body& self, const string& name){ return self.findDevice(name); })
        .def("addDevice", (void(Body::*)(Device*, Link*)) &Body::addDevice)
        .def("initializeDeviceStates", &Body::initializeDeviceStates)
        .def("removeDevice", &Body::removeDevice)
        .def("removeDevicesOfLink", &Body::removeDevicesOfLink)
        .def("clearDevices", &Body::clearDevices)
        .def("isStaticModel", &Body::isStaticModel)
        .def("isFixedRootModel", &Body::isFixedRootModel)
        .def("setRootLinkFixed", &Body::setRootLinkFixed)
        .def("resetDefaultPosition",
             [](Body& self, const python::Matrix4RMArg& T){ Isometry3 iT(T.value); self.resetDefaultPosition(iT); })
        .def_prop_ro("defaultPosition", [](Body& self){ Isometry3 res(self.defaultPosition()); return res.matrix(); })
        .def_prop_ro("mass", &Body::mass)
        .def("calcCenterOfMass", &Body::calcCenterOfMass)
        .def_prop_ro("centerOfMass", &Body::centerOfMass)
        .def("calcTotalMomentum", [](Body& self){
            Vector3 P, L;
            self.calcTotalMomentum(P, L);
            return nb::make_tuple(P, L);
            })
        .def("calcForwardKinematics",
             &Body::calcForwardKinematics, nb::arg("calcVelocity") = false, nb::arg("calcAcceleration") = false)
        .def("clearExternalForces", &Body::clearExternalForces)
        .def_prop_ro("numExtraJoints", &Body::numExtraJoints)
        .def("clearExtraJoints", &Body::clearExtraJoints)
        .def_prop_ro("info", (Mapping*(Body::*)()) &Body::info)
        .def("getInfo", Body_getInfo)
        .def("hasVirtualJointForces", &Body::hasVirtualJointForces)
        .def("setVirtualJointForces", &Body::setVirtualJointForces)
        .def_static("addCustomizerDirectory", &Body::addCustomizerDirectory)
        .def("resetLinkName", &Body::resetLinkName)
        .def("resetJointSpecificName", (void(Body::*)(Link*)) &Body::resetLinkName)
        .def("resetJointSpecificName", (void(Body::*)(Link*, const std::string& name)) &Body::resetLinkName)
        ;

    nb::class_<ExtraJoint, Referenced> extraJoint(m, "ExtraJoint");
    extraJoint
        .def(nb::init<>())
        .def(nb::init<ExtraJoint::ExtraJointType>())
        .def("__init__", [](ExtraJoint* self, ExtraJoint::ExtraJointType type, const python::Vector3Arg& axis){
            new(self) ExtraJoint(type, axis.value); })
        .def("setType", &ExtraJoint::setType)
        .def("setLink", &ExtraJoint::setLink)
        .def("setLocalPosition",
             [](ExtraJoint& self, int which, const python::Matrix4RMArg& T){ self.setLocalPosition(which, Isometry3(T.value)); })
        .def("setLocalRotation",
             [](ExtraJoint& self, int which, const python::Matrix3RMArg& R){ self.setLocalRotation(which, R.value); })
        .def("setLocalTranslation",
             [](ExtraJoint& self, int which, const python::Vector3Arg& p){ self.setLocalTranslation(which, p.value); })
        .def("setAxis", [](ExtraJoint& self, const python::Vector3Arg& a){ self.setAxis(a.value); })
        .def("setPoint", &ExtraJoint::setPoint)
        ;

    nb::enum_<ExtraJoint::ExtraJointType>(extraJoint, "ExtraJointType", nb::is_arithmetic())
        .value("Fixed", ExtraJoint::ExtraJointType::Fixed)
        .value("Hinge", ExtraJoint::ExtraJointType::Hinge)
        .value("Ball", ExtraJoint::ExtraJointType::Ball)
        .value("Piston", ExtraJoint::ExtraJointType::Piston)
        .value("EJ_BALL", ExtraJoint::ExtraJointType::Ball)
        .value("EJ_PISTON", ExtraJoint::ExtraJointType::Piston)
        .export_values();
}

}
