/*!
  @author Shin'ichiro Nakaoka
 */

#include "../Body.h"
#include "../BodyLoader.h"
#include "../BodyMotion.h"
#include "../InverseKinematics.h"
#include "../JointPath.h"
#include "../LeggedBodyHelper.h"
#include <cnoid/PyUtil>
#include <pybind11/operators.h>

using namespace std;
using namespace cnoid;
namespace py = pybind11;

namespace {

using Matrix4RM = Eigen::Matrix<double, 4, 4, Eigen::RowMajor>;

}

namespace cnoid {

void exportPyBody(py::module& m);
void exportPyLink(py::module& m);
void exportPyDeviceTypes(py::module& m);
void exportPyMaterial(py::module& m);

}

PYBIND11_MODULE(Body, m)
{
    m.doc() = "Choreonoid Body module";

    py::module::import("cnoid.Util");

    exportPyBody(m);
    exportPyLink(m);
    exportPyDeviceTypes(m);
    exportPyMaterial(m);

    py::class_<AbstractBodyLoader>(m, "AbstractBodyLoader")
        .def("setVerbose", &AbstractBodyLoader::setVerbose)
        .def("setShapeLoadingEnabled", &AbstractBodyLoader::setShapeLoadingEnabled)
        .def("setDefaultDivisionNumber", &AbstractBodyLoader::setDefaultDivisionNumber)
        .def("setDefaultCreaseAngle", &AbstractBodyLoader::setDefaultCreaseAngle)
        .def("load", &AbstractBodyLoader::load)
        ;

    py::class_<BodyLoader, AbstractBodyLoader>(m, "BodyLoader")
        .def(py::init<>())
        .def("load", (Body*(BodyLoader::*)(const string&))&BodyLoader::load)
        .def("lastActualBodyLoader", &BodyLoader::lastActualBodyLoader)
        ;

    py::class_<JointPath, shared_ptr<JointPath>>(m, "JointPath")
        .def(py::init<>())
        .def_static("getCustomPath", &JointPath::getCustomPath)
        .def_property_readonly("empty", &JointPath::empty)
        .def_property_readonly("size", &JointPath::size)
        .def_property_readonly("numJoints", &JointPath::numJoints)
        .def("joint", &JointPath::joint)
        .def_property_readonly("baseLink", &JointPath::baseLink)
        .def_property_readonly("endLink", &JointPath::endLink)
        .def("isJointDownward", &JointPath::isJointDownward)
        .def("calcForwardKinematics", &JointPath::calcForwardKinematics,
             py::arg("calcVelocity") = false, py::arg("calcAcceleration") = false)
        .def("indexOf", &JointPath::indexOf)
        .def("isCustomIkDisabled", &JointPath::isCustomIkDisabled)
        .def("setCustomIkDisabled", &JointPath::setCustomIkDisabled)
        .def("isBestEffortIkMode", &JointPath::isBestEffortIkMode)
        .def("setBestEffortIkMode", &JointPath::setBestEffortIkMode)
        .def("setNumericalIkMaxIkError", &JointPath::setNumericalIkMaxIkError)
        .def("setNumericalIkDeltaScale", &JointPath::setNumericalIkDeltaScale)
        .def("setNumericalIkMaxIterations", &JointPath::setNumericalIkMaxIterations)
        .def("setNumericalIkDampingConstant", &JointPath::setNumericalIkDampingConstant)
        .def_property_readonly("numericalIkDefaultDeltaScale", &JointPath::numericalIkDefaultDeltaScale)
        .def_property_readonly("numericalIkDefaultMaxIterations", &JointPath::numericalIkDefaultMaxIterations)
        .def_property_readonly("numericalIkDefaultMaxIkError", &JointPath::numericalIkDefaultMaxIkError)
        .def_property_readonly("numericalIkDefaultDampingConstant", &JointPath::numericalIkDefaultDampingConstant)
        .def("customizeTarget", &JointPath::customizeTarget)
        .def("calcInverseKinematics", (bool(JointPath::*)())&JointPath::calcInverseKinematics)
        .def("setBaseLinkGoal",
             [](JointPath& self, Eigen::Ref<Matrix4RM> T) -> JointPath& { return self.setBaseLinkGoal(Isometry3(T)); })
        .def("calcInverseKinematics",
             [](JointPath& self, Eigen::Ref<const Matrix4RM> T){ return self.calcInverseKinematics(Isometry3(T)); })
        .def("calcRemainingPartForwardKinematicsForInverseKinematics",
             &JointPath::calcRemainingPartForwardKinematicsForInverseKinematics)
        .def_property_readonly("numIterations", &JointPath::numIterations)
        .def("hasCustomIK", &JointPath::hasCustomIK)
        .def_property("name", &JointPath::name, &JointPath::setName)
        .def("setName", &JointPath::setName)

        // deprecated
        .def("getNumJoints", &JointPath::numJoints)
        .def("getJoint", &JointPath::joint)
        .def("getBaseLink", &JointPath::baseLink)
        .def("getEndLink", &JointPath::endLink)
        .def("getIndexOf", &JointPath::indexOf)
        .def("getNumIterations", &JointPath::numIterations)
        ;

    m.def("getCustomJointPath", getCustomJointPath);

    py::class_<BodyMotion, shared_ptr<BodyMotion>> bodyMotion(m, "BodyMotion");
    bodyMotion
        .def_property("numJoints", &BodyMotion::numJoints, &BodyMotion::setNumParts)
        .def("setNumJoints", &BodyMotion::setNumParts)
        .def_property_readonly("numLinks", &BodyMotion::numLinks)
        .def_property("frameRate", &BodyMotion::frameRate, &BodyMotion::setFrameRate)
        .def("setFrameRate", &BodyMotion::setFrameRate)
        .def("getOffsetTimeFrame", &BodyMotion::getOffsetTimeFrame)
        .def_property("numFrames", &BodyMotion::numFrames, &BodyMotion::setNumFrames)
        .def("setNumFrames", &BodyMotion::setNumFrames)
        .def_property_readonly("jointPosSeq", [](BodyMotion& self){ return self.jointPosSeq(); })
        .def_property_readonly("linkPosSeq", [](BodyMotion& self){ return self.linkPosSeq(); })
        .def("frame", [](BodyMotion& self, int f){ return self.frame(f); })

        // deprecated
        .def("getNumJoints", &BodyMotion::numJoints)
        .def("getNumLinks", &BodyMotion::numLinks)
        .def("getFrameRate",&BodyMotion::getFrameRate)
        .def("getNumFrames", &BodyMotion::numFrames)
        .def("getJointPosSeq", [](BodyMotion& self){ return self.jointPosSeq(); })
        .def("getLinkPosSeq", [](BodyMotion& self){ return self.linkPosSeq(); })
        .def("getFrame", [](BodyMotion& self, int f){ return self.frame(f); })
        .def("setNumParts", &BodyMotion::setNumJoints)
        .def("getNumParts", &BodyMotion::numJoints)
        ;

    py::class_<BodyMotion::Frame>(m, "Frame")
        .def("frame", &BodyMotion::Frame::frame)
        .def(py::self << Body())
        .def(Body() >> py::self)

        // deprecated
        .def("getFrame", &BodyMotion::Frame::frame)
        ;

    py::class_<LeggedBodyHelper>(m, "LeggedBodyHelper")
        .def(py::init<>())
        .def(py::init<Body*>())
        .def_property_readonly("numFeet", &LeggedBodyHelper::numFeet)
        .def("footLink", &LeggedBodyHelper::footLink)
        ;
}
