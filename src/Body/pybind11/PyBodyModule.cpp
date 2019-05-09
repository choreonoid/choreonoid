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

    py::class_<JointPath, shared_ptr<JointPath>>(m, "JointPath")
        .def(py::init<>())
        .def_property_readonly("numJoints", &JointPath::numJoints)
        .def("joint", &JointPath::joint)
        .def_property_readonly("baseLink", &JointPath::baseLink)
        .def_property_readonly("endLink", &JointPath::endLink)
        .def("indexOf", &JointPath::indexOf)
        .def("customizeTarget", &JointPath::customizeTarget)
        .def_property_readonly("numIterations", &JointPath::numIterations)
        .def("calcJacobian", &JointPath::calcJacobian)
        .def("calcInverseKinematics", (bool(JointPath::*)())&JointPath::calcInverseKinematics)
        .def("calcInverseKinematics", (bool(JointPath::*)(const Position&))&JointPath::calcInverseKinematics)

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
}
