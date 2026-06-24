#include "../Body.h"
#include "../BodyLoader.h"
#include "../BodyMotion.h"
#include "../BodyState.h"
#include "../BodyStateSeq.h"
#include "../InverseKinematics.h"
#include "../JointPath.h"
#include "../LeggedBodyHelper.h"
#include <cnoid/PyUtil>
#include <cnoid/PyEigenTypes>
#include <nanobind/stl/shared_ptr.h>

using namespace std;
using namespace cnoid;
namespace nb = nanobind;

namespace cnoid {

void exportPyBody(nb::module_& m);
void exportPyLink(nb::module_& m);
void exportPyDeviceTypes(nb::module_& m);
void exportPyMaterial(nb::module_& m);

}

NB_MODULE(Body, m)
{
    m.doc() = "Choreonoid Body module";

    nb::module_::import_("cnoid.Util");

    exportPyBody(m);
    exportPyLink(m);
    exportPyDeviceTypes(m);
    exportPyMaterial(m);

    nb::class_<AbstractBodyLoader>(m, "AbstractBodyLoader")
        .def("setVerbose", &AbstractBodyLoader::setVerbose)
        .def("setShapeLoadingEnabled", &AbstractBodyLoader::setShapeLoadingEnabled)
        .def("setDefaultDivisionNumber", &AbstractBodyLoader::setDefaultDivisionNumber)
        .def("setDefaultCreaseAngle", &AbstractBodyLoader::setDefaultCreaseAngle)
        .def("load", &AbstractBodyLoader::load)
        ;

    nb::class_<BodyLoader, AbstractBodyLoader>(m, "BodyLoader")
        .def(nb::init<>())
        .def("load", (Body*(BodyLoader::*)(const string&)) &BodyLoader::load)
        .def("lastActualBodyLoader", &BodyLoader::lastActualBodyLoader)
        ;

    nb::class_<JointPath>(m, "JointPath")
        .def(nb::init<>())
        .def_static("getCustomPath",
                    [](Link* baseLink, Link* endLink){ return JointPath::getCustomPath(baseLink, endLink); })
        .def_prop_ro("empty", &JointPath::empty)
        .def_prop_ro("size", &JointPath::size)
        .def_prop_ro("numJoints", &JointPath::numJoints)
        .def("joint", &JointPath::joint)
        .def_prop_ro("baseLink", &JointPath::baseLink)
        .def_prop_ro("endLink", &JointPath::endLink)
        .def("isJointDownward", &JointPath::isJointDownward)
        .def("calcForwardKinematics", &JointPath::calcForwardKinematics,
             nb::arg("calcVelocity") = false, nb::arg("calcAcceleration") = false)
        .def("indexOf", &JointPath::indexOf)
        .def("isCustomIkDisabled", &JointPath::isCustomIkDisabled)
        .def("setCustomIkDisabled", &JointPath::setCustomIkDisabled)
        .def("isBestEffortIkMode", &JointPath::isBestEffortIkEnabled)
        .def("setBestEffortIkMode", &JointPath::setBestEffortIkEnabled)
        .def("setNumericalIkMaxIkError", &JointPath::setNumericalIkMaxIkError)
        .def("setNumericalIkDeltaScale", &JointPath::setNumericalIkDeltaScale)
        .def("setNumericalIkMaxIterations", &JointPath::setNumericalIkMaxIterations)
        .def("setNumericalIkDampingConstant", &JointPath::setNumericalIkDampingConstant)
        .def("isNumericalIkExactEndPositionEnabled", &JointPath::isNumericalIkExactEndPositionEnabled)
        .def("setNumericalIkExactEndPositionEnabled", &JointPath::setNumericalIkExactEndPositionEnabled)
        .def_prop_ro("numericalIkDefaultDeltaScale", &JointPath::numericalIkDefaultDeltaScale)
        .def_prop_ro("numericalIkDefaultMaxIterations", &JointPath::numericalIkDefaultMaxIterations)
        .def_prop_ro("numericalIkDefaultMaxIkError", &JointPath::numericalIkDefaultMaxIkError)
        .def_prop_ro("numericalIkDefaultDampingConstant", &JointPath::numericalIkDefaultDampingConstant)
        .def("customizeTarget", &JointPath::customizeTarget)
        .def("calcInverseKinematics", (bool(JointPath::*)()) &JointPath::calcInverseKinematics)
        .def("setBaseLinkGoal",
             [](JointPath& self, const python::Matrix4RMArg& T) -> JointPath& { return self.setBaseLinkGoal(Isometry3(T.value)); },
             nb::rv_policy::reference)
        .def("calcInverseKinematics",
             [](JointPath& self, const python::Matrix4RMArg& T){ return self.calcInverseKinematics(Isometry3(T.value)); })
        .def("calcRemainingPartForwardKinematicsForInverseKinematics",
             &JointPath::calcRemainingPartForwardKinematicsForInverseKinematics)
        .def_prop_ro("numIterations", &JointPath::numIterations)
        .def("hasCustomIK", &JointPath::hasCustomIK)
        .def_prop_rw("name", &JointPath::name, &JointPath::setName)
        .def("setName", &JointPath::setName)
        ;

    nb::class_<BodyMotion> bodyMotion(m, "BodyMotion");
    bodyMotion
        .def("cloneSeq", &BodyMotion::cloneSeq)
        .def_prop_rw("frameRate", &BodyMotion::frameRate, &BodyMotion::setFrameRate)
        .def_prop_ro("timeStep", &BodyMotion::timeStep)
        .def_prop_rw("offsetTime", &BodyMotion::offsetTime, &BodyMotion::setOffsetTime)
        .def_prop_rw("numFrames", &BodyMotion::numFrames, &BodyMotion::setNumFrames)
        .def_prop_ro("stateSeq", [](BodyMotion& self){ return self.stateSeq(); })
        .def("getFrame", [](BodyMotion& self, int f){ return self.frame(f); })
        .def("load", [](BodyMotion& self, const std::string& filename){ return self.load(filename); })
        .def("save", [](BodyMotion& self, const std::string& filename){ return self.save(filename); })

        // AbstractSeq members
        .def("getFrameRate", &BodyMotion::frameRate)
        .def("setFrameRate", &BodyMotion::setFrameRate)
        .def("getOffsetTimeFrame", &BodyMotion::getOffsetTimeFrame)
        .def("getNumFrames", &BodyMotion::numFrames)
        .def("setNumFrames", &BodyMotion::setNumFrames)

        // AbstractMultiSeq members
        .def("setNumParts", &BodyMotion::setNumJoints)
        .def("getNumParts", &BodyMotion::numJoints)
        ;

    nb::class_<BodyMotion::Frame>(m, "Frame")
        .def("frame", &BodyMotion::Frame::frame)
        .def("__lshift__", [](BodyMotion::Frame& self, const Body& body){ return self << body; })
        .def("__rrshift__",
             [](BodyMotion::Frame& self, Body& body) -> Body& { body >> self; return body; },
             nb::rv_policy::reference)
        ;

    // BodyState is exposed as a plain value type. The block/view types
    // (BodyStateBlock, LinkPosition) are not exposed yet; only the operations
    // needed to capture and restore a body's kinematic state are provided.
    // The numXxx/hasXxx accessors read the data header directly, which is out
    // of range when the state is empty, so they are guarded here.
    nb::class_<BodyState>(m, "BodyState")
        .def(nb::init<>())
        .def(nb::init<const Body*>())
        .def("clear", &BodyState::clear)
        .def_prop_ro("empty", &BodyState::empty)
        .def_prop_ro("numLinkPositions",
                     [](const BodyState& self){ return self.empty() ? 0 : self.numLinkPositions(); })
        .def_prop_ro("numJointDisplacements",
                     [](const BodyState& self){ return self.empty() ? 0 : self.numJointDisplacements(); })
        .def_prop_ro("numDeviceStates",
                     [](const BodyState& self){ return self.empty() ? 0 : self.numDeviceStates(); })
        .def("hasLinkPositions",
             [](const BodyState& self){ return !self.empty() && self.hasLinkPositions(); })
        .def("hasJointDisplacements",
             [](const BodyState& self){ return !self.empty() && self.hasJointDisplacements(); })
        .def("hasDeviceStates",
             [](const BodyState& self){ return !self.empty() && self.hasDeviceStates(); })
        .def("storeStateOfBody", [](BodyState& self, const Body* body){ self.storeStateOfBody(body); })
        .def("restoreStateToBody", [](const BodyState& self, Body* body){ return self.restoreStateToBody(body); })
        .def("storePositionOfBody", [](BodyState& self, const Body* body){ self.storePositionOfBody(body); })
        .def("restorePositionToBody", [](const BodyState& self, Body* body){ return self.restorePositionToBody(body); })
        ;

    nb::class_<BodyStateSeq>(m, "BodyStateSeq")
        .def_prop_ro("numLinkPositionsHint", &BodyStateSeq::numLinkPositionsHint)
        .def("setNumLinkPositionsHint", &BodyStateSeq::setNumLinkPositionsHint)
        .def_prop_ro("numJointDisplacementsHint", &BodyStateSeq::numJointDisplacementsHint)
        .def("setNumJointDisplacementsHint", &BodyStateSeq::setNumJointDisplacementsHint)
        ;

    nb::class_<LeggedBodyHelper>(m, "LeggedBodyHelper")
        .def(nb::init<>())
        .def(nb::init<Body*>())
        .def_prop_ro("numFeet", &LeggedBodyHelper::numFeet)
        .def("footLink", &LeggedBodyHelper::footLink)
        ;
}
