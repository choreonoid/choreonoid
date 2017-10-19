/*!
  @author Shin'ichiro Nakaoka
 */

#include "../Body.h"
#include "../BodyMotion.h"
#include <cnoid/ValueTree>
#include <cnoid/PyReferenced>
#include <cnoid/PyEigenTypes>
#include <pybind11/operators.h>

using namespace std;
using namespace cnoid;
namespace py = pybind11;

namespace cnoid {

void exportPyBody(py::module& m)
{
    py::class_<Body, BodyPtr, Referenced> body(m, "Body");
    body
        .def("clone", &Body::clone)
        .def("createLink", &Body::createLink)
        .def("createLink", [](Body& self){ return self.createLink(); })
        .def("name", &Body::name)
        .def("setName", &Body::setName)
        .def("modelName", &Body::modelName)
        .def("setModelName", &Body::setModelName)
        .def("setRootLink", &Body::setRootLink)
        .def("updateLinkTree", &Body::updateLinkTree)
        .def("initializeState", &Body::initializeState)
        .def("numJoints", &Body::numJoints)
        .def("numVirtualJoints", &Body::numVirtualJoints)
        .def("numAllJoints", &Body::numAllJoints)
        .def("joint", &Body::joint)
        .def("numLinks", &Body::numLinks)
        .def("link", (Link*(Body::*)(int)const)&Body::link)
        .def("link", (Link*(Body::*)(const string&)const)&Body::link)
        .def("rootLink", &Body::rootLink)
        .def("numDevices", &Body::numDevices)
        .def("device", &Body::device)
        .def("addDevice", &Body::addDevice)
        .def("initializeDeviceStates", &Body::initializeDeviceStates)
        .def("clearDevices", &Body::clearDevices)
        .def("isStaticModel", &Body::isStaticModel)
        .def("isFixedRootModel", &Body::isFixedRootModel)
        .def("resetDefaultPosition", &Body::resetDefaultPosition)
        .def("defaultPosition", &Body::defaultPosition)
        .def("mass", &Body::mass)
        .def("calcCenterOfMass", &Body::calcCenterOfMass)
        .def("centerOfMass", &Body::centerOfMass)
        .def("calcTotalMomentum", [](Body& self) {
            Vector3 P, L;
            self.calcTotalMomentum(P, L);
            return py::make_tuple(P, L);
            })
        .def("calcForwardKinematics", &Body::calcForwardKinematics)
        .def("calcForwardKinematics", [](Body& self, bool calcVelocity){ self.calcForwardKinematics(calcVelocity); })
        .def("calcForwardKinematics", [](Body& self){ self.calcForwardKinematics(); })
        .def("clearExternalForces", &Body::clearExternalForces)
        .def("numExtraJoints", &Body::numExtraJoints)
        .def("clearExtraJoints", &Body::clearExtraJoints)
        .def("hasVirtualJointForces", &Body::hasVirtualJointForces)
        .def("setVirtualJointForces", &Body::setVirtualJointForces)
        .def_static("addCustomizerDirectory", &Body::addCustomizerDirectory)
        .def(BodyMotion::Frame() >> py::self)
        ;

    py::class_<ExtraJoint> extraJoint(m, "ExtraJoint");
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
}
   
}
