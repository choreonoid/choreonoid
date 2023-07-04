#include "../Body.h"
#include "../BodyMotion.h"
#include "PyDeviceList.h"
#include <cnoid/ValueTree>
#include <cnoid/CloneMap>
#include <cnoid/PyUtil>
#include <pybind11/operators.h>

using namespace std;
using namespace cnoid;
namespace py = pybind11;

namespace {

using Matrix4RM = Eigen::Matrix<double, 4, 4, Eigen::RowMajor>;
using Matrix3RM = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>;

py::object Body_getInfo(Body& self, const std::string& key, py::object defaultValue)
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

void exportPyBody(py::module& m)
{
    py::class_<Body, BodyPtr, Referenced> body(m, "Body");
    body
        .def(py::init<>())
        .def(py::init<const std::string&>())
        .def("__repr__", [](const Body &self) { return "<cnoid.Body.Body named '" + self.name() + "'>"; })
        .def("clone", (Body*(Body::*)()const) &Body::clone)
        .def("createLink", &Body::createLink, py::arg("org") = nullptr, py::arg("cloneMap") = nullptr)
        .def_property("name", &Body::name, &Body::setName)
        .def("setName", &Body::setName)
        .def_property("modelName", &Body::modelName, &Body::setModelName)
        .def("setModelName", &Body::setModelName)
        .def("setRootLink", &Body::setRootLink)
        .def("updateLinkTree", &Body::updateLinkTree)
        .def("initializePosition", &Body::initializePosition)
        .def("initializeState", &Body::initializeState)
        .def_property_readonly("parentBody", &Body::parentBody)
        .def_property_readonly("parentBodyLink", &Body::parentBodyLink)
        .def("setParent", &Body::setParent)
        .def("resetParent", &Body::resetParent)
        .def("syncPositionWithParentBody",
             &Body::syncPositionWithParentBody, py::arg("doForwardKinematics") = true)
        .def_property_readonly("numLinks", &Body::numLinks)
        .def("link", (Link*(Body::*)(int)const)&Body::link)
        .def("link", (Link*(Body::*)(const string&)const)&Body::link)
        .def("linkTraverse", &Body::linkTraverse)
        .def_property_readonly("links", &Body::links)
        .def_property_readonly("rootLink", &Body::rootLink)
        .def("findUniqueEndLink", &Body::findUniqueEndLink)
        .def_property_readonly("lastSerialLink", &Body::lastSerialLink)
        .def_property_readonly("numJoints", &Body::numJoints)
        .def_property_readonly("numVirtualJoints", &Body::numVirtualJoints)
        .def_property_readonly("numAllJoints", &Body::numAllJoints)
        .def("joint", (Link*(Body::*)(int)const)&Body::joint)
        .def("joint", (Link*(Body::*)(const string&)const)&Body::joint)
        .def_property_readonly("joints",[](Body& self){
             auto joints = self.allJoints();
             joints.resize(self.numJoints());
             return joints; })
        .def_property_readonly("allJoints", &Body::allJoints)
        .def_property_readonly("numDevices", &Body::numDevices)
        .def_property_readonly("devices", [](Body& self) { return getPyDeviceList(self.devices()); })
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
        .def("resetDefaultPosition", &Body::resetDefaultPosition)
        .def_property_readonly("defaultPosition", &Body::defaultPosition)
        .def_property_readonly("mass", &Body::mass)
        .def("calcCenterOfMass", &Body::calcCenterOfMass)
        .def_property_readonly("centerOfMass", &Body::centerOfMass)
        .def("calcTotalMomentum", [](Body& self) {
            Vector3 P, L;
            self.calcTotalMomentum(P, L);
            return py::make_tuple(P, L);
            })
        .def("calcForwardKinematics",
             &Body::calcForwardKinematics, py::arg("calcVelocity") = false, py::arg("calcAcceleration") = false)
        .def("clearExternalForces", &Body::clearExternalForces)
        .def_property_readonly("numExtraJoints", &Body::numExtraJoints)
        .def("clearExtraJoints", &Body::clearExtraJoints)
        .def_property_readonly("info", (Mapping*(Body::*)())&Body::info)
        .def("getInfo", Body_getInfo)
        .def("hasVirtualJointForces", &Body::hasVirtualJointForces)
        .def("setVirtualJointForces", &Body::setVirtualJointForces)
        .def_static("addCustomizerDirectory", &Body::addCustomizerDirectory)
        .def("resetLinkName", &Body::resetLinkName)
        .def("resetJointSpecificName", (void(Body::*)(Link *)) &Body::resetLinkName)
        .def("resetJointSpecificName", (void(Body::*)(Link *, const std::string &name)) &Body::resetLinkName)

        // deprecated
        .def("getName", &Body::name)
        .def("getModelName", &Body::modelName)
        .def("getNumJoints", &Body::numJoints)
        .def("getNumVirtualJoints", &Body::numVirtualJoints)
        .def("getNumAllJoints", &Body::numAllJoints)
        .def("getJoint", (Link*(Body::*)(int)const)&Body::joint)
        .def("getNumLinks", &Body::numLinks)
        .def("getLink", (Link*(Body::*)(int)const)&Body::link)
        .def("getLink", (Link*(Body::*)(const string&)const)&Body::link)
        .def("getRootLink", &Body::rootLink)
        .def("getNumDevices", &Body::numDevices)
        .def("getDevice", &Body::device)
        .def("getDefaultPosition", &Body::defaultPosition)
        .def("getMass", &Body::mass)
        .def("getCenterOfMass", &Body::centerOfMass)
        .def("getNumExtraJoints", &Body::numExtraJoints)
        ;

    py::class_<ExtraJoint, ExtraJointPtr, Referenced> extraJoint(m, "ExtraJoint");
    extraJoint
        .def(py::init<>())
        .def(py::init<ExtraJoint::ExtraJointType>())
        .def(py::init<ExtraJoint::ExtraJointType, const Vector3&>())
        .def("setType", &ExtraJoint::setType)
        .def("setLink", &ExtraJoint::setLink)
        .def("setLocalPosition",
             [](ExtraJoint& self, int which, Eigen::Ref<const Matrix4RM> T){ self.setLocalPosition(which, Isometry3(T)); })
        .def("setLocalRotation",
             [](ExtraJoint& self, int which, Eigen::Ref<const Matrix3RM> R){ self.setLocalRotation(which, R); })
        .def("setLocalTranslation",
             [](ExtraJoint& self, int which, const Vector3& p){ self.setLocalTranslation(which, p); })
        .def("setAxis", [](ExtraJoint& self, int which, const Vector3& a){ self.setAxis(a); })
        .def("setPoint", &ExtraJoint::setPoint)
        ;

    py::enum_<ExtraJoint::ExtraJointType>(extraJoint, "ExtraJointType")
        .value("Fixed", ExtraJoint::ExtraJointType::Fixed)
        .value("Hinge", ExtraJoint::ExtraJointType::Hinge)
        .value("Ball", ExtraJoint::ExtraJointType::Ball)
        .value("Piston", ExtraJoint::ExtraJointType::Piston)
        .value("EJ_BALL", ExtraJoint::ExtraJointType::Ball)
        .value("EJ_PISTON", ExtraJoint::ExtraJointType::Piston)
        .export_values();
}
   
}
