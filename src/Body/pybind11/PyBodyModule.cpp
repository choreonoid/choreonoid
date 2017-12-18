/*!
  @author Shin'ichiro Nakaoka
 */

#include "../Body.h"
#include "../BodyLoader.h"
#include "../BodyMotion.h"
#include "../InverseKinematics.h"
#include "../JointPath.h"
#include <cnoid/PyEigenTypes>
#include <pybind11/operators.h>

using namespace std;
using namespace cnoid;
namespace py = pybind11;

namespace cnoid {

void exportPyBody(py::module& m);
void exportPyLink(py::module& m);
void exportPyDeviceTypes(py::module& m);

}

PYBIND11_MODULE(Body, m)
{
    m.doc() = "Choreonoid Body module";

    py::module::import("cnoid.Util");

    exportPyBody(m);
    exportPyLink(m);
    exportPyDeviceTypes(m);

    py::class_<AbstractBodyLoader>(m, "AbstractBodyLoader")
        .def("setVerbose", &AbstractBodyLoader::setVerbose)
        .def("setShapeLoadingEnabled", &AbstractBodyLoader::setShapeLoadingEnabled)
        .def("setDefaultDivisionNumber", &AbstractBodyLoader::setDefaultDivisionNumber)
        .def("setDefaultCreaseAngle", &AbstractBodyLoader::setDefaultCreaseAngle)
        .def("load", &AbstractBodyLoader::load)
        ;

    py::class_<BodyLoader, AbstractBodyLoader>(m, "BodyLoader")
        .def("load", (Body*(BodyLoader::*)(const string&))&BodyLoader::load)
        .def("lastActualBodyLoader", &BodyLoader::lastActualBodyLoader)
        ;

    py::class_<JointPath, JointPathPtr>(m, "JointPath")
        .def(py::init<>())
        .def("numJoints", &JointPath::numJoints)
        .def("joint", &JointPath::joint)
        .def("baseLink", &JointPath::baseLink)
        .def("endLink", &JointPath::endLink)
        .def("indexOf", &JointPath::indexOf)
        .def("customizeTarget", &JointPath::customizeTarget)
        .def("numIterations", &JointPath::numIterations)
        .def("calcJacobian", &JointPath::calcJacobian)
        .def("calcInverseKinematics", (bool(JointPath::*)())&JointPath::calcInverseKinematics)
        .def("calcInverseKinematics", (bool(JointPath::*)(const Position&))&JointPath::calcInverseKinematics)
        ;

    m.def("getCustomJointPath", getCustomJointPath);

    py::class_<BodyMotion, BodyMotionPtr> bodyMotion(m, "BodyMotion");
    bodyMotion
        .def("setNumJoints", &BodyMotion::setNumParts)
        .def("numJoints", &BodyMotion::numJoints)
        .def("numLings", &BodyMotion::numLinks)
        .def("frameRate", &BodyMotion::frameRate)
        .def("getFrameRate",&BodyMotion::getFrameRate)
        .def("setFrameRate", &BodyMotion::setFrameRate)
        .def("getOffsetTimeFrame", &BodyMotion::getOffsetTimeFrame)
        .def("numFrames", &BodyMotion::numFrames)
        .def("getNumFrames", &BodyMotion::getNumFrames)
        .def("setNumFrames", &BodyMotion::setNumFrames)
        .def("jointPosSeq", (MultiValueSeqPtr(BodyMotion::*)())&BodyMotion::jointPosSeq)
        .def("linkPosSeq", (MultiSE3SeqPtr(BodyMotion::*)())&BodyMotion::linkPosSeq)
        .def("frame", (BodyMotion::Frame (BodyMotion::*)(int)) &BodyMotion::frame)
        // deprecated
        .def("setNumParts", &BodyMotion::setNumJoints)
        .def("getNumParts", &BodyMotion::numJoints)
        ;

    py::class_<BodyMotion::Frame>(m, "Frame")
        .def("frame", &BodyMotion::Frame::frame)
        .def(py::self << Body())
        .def(Body() >> py::self)
        ;
}
