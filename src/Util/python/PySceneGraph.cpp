#include "PyUtil.h"
#include "PyNumpyHelper.h"
#include "PyEigenTypes.h"
#include "../SceneGraph.h"
#include "../SceneCameras.h"
#include "../SceneLights.h"
#include "../CloneMap.h"
#include <nanobind/eigen/dense.h>

using namespace cnoid;
namespace nb = nanobind;

namespace cnoid {

void exportPySceneGraph(nb::module_& m)
{
    nb::class_<SgUpdate> sgUpdate(m, "SgUpdate");

    sgUpdate
        .def(nb::init<>())
        .def(nb::init<int>())
        .def_prop_rw("action", &SgUpdate::action, &SgUpdate::setAction)
        .def("setAction", &SgUpdate::setAction)
        ;

    nb::class_<SgUpdateRef>(m, "SgUpdateRef")
        .def(nb::init<>())
        .def(nb::init<SgUpdate&>())
        .def(nb::init<const SgUpdateRef&>())
        .def(nb::init<bool>())
        ;

    nb::implicitly_convertible<SgUpdate, SgUpdateRef>();
    nb::implicitly_convertible<bool, SgUpdateRef>();

    nb::enum_<SgUpdate::Action>(sgUpdate, "Action", nb::is_arithmetic())
        .value("None", SgUpdate::None)
        .value("Added", SgUpdate::Added)
        .value("Removed", SgUpdate::Removed)
        .value("Modified", SgUpdate::Modified)
        .export_values();

    nb::class_<SgObject, Referenced>(m, "SgObject")
        .def_prop_rw("name", &SgObject::name, (void(SgObject::*)(const std::string&)) &SgObject::setName)
        .def("setName", (void(SgObject::*)(const std::string&)) &SgObject::setName)
        .def("notifyUpdate",(void(SgObject::*)(SgUpdate&)) &SgObject::notifyUpdate)
        .def("notifyUpdate",(void(SgObject::*)(int)) &SgObject::notifyUpdate, nb::arg("action") = SgUpdate::Modified)
        ;

    nb::class_<SgNode, SgObject>(m, "SgNode")
        .def(nb::init<>())
        .def(nb::init<const SgNode&>())
        .def("isGroupNode", &SgNode::isGroupNode)
        .def("boundingBox", [](SgNode &self) { BoundingBox ret = self.boundingBox(); return ret; })
        .def("untransformedBoundingBox", [](SgNode &self) { BoundingBox ret = self.untransformedBoundingBox(); return ret; })
        ;

    nb::class_<SgGroup, SgNode>(m, "SgGroup")
        .def(nb::init<>())
        .def(nb::init<const SgGroup&>())
        .def_prop_ro("empty", &SgGroup::empty)
        .def_prop_ro("numChildren", &SgGroup::numChildren)
        .def("clearChildren", &SgGroup::clearChildren, nb::arg("update") = false)
        .def("child", (SgNode*(SgGroup::*)(int)) &SgGroup::child)
        .def("addChild",
             (void(SgGroup::*)(SgNode*, SgUpdateRef)) &SgGroup::addChild,
             nb::arg("node"), nb::arg("update") = false)
        ;

    nb::class_<SgTransform, SgGroup>(m, "SgTransform");

    nb::class_<SgPosTransform, SgTransform>(m, "SgPosTransform")
        .def(nb::init<>())
        .def("__init__", [](SgPosTransform* self, const python::Matrix4RMArg& T){ new (self) SgPosTransform(Isometry3(T.value)); })
        .def(nb::init<const SgPosTransform&>())

        .def_prop_rw(
            "T",
            [](SgPosTransform& self) -> Isometry3::MatrixType& { return self.T().matrix(); },
            [](SgPosTransform& self, const python::Matrix4RMArg& T){ self.setPosition(T.value); })
        .def_prop_rw(
            "position",
            [](SgPosTransform& self) -> Isometry3::MatrixType& { return self.T().matrix(); },
            [](SgPosTransform& self, const python::Matrix4RMArg& T){ self.setPosition(T.value); })
        .def("setPosition",
             [](SgPosTransform& self, const python::Matrix4RMArg& T){ self.setPosition(T.value); })
        .def_prop_rw(
            "translation",
            [](SgPosTransform& self){ return python::eigenVectorView(self.translation(), nb::find(&self)); },
            [](SgPosTransform& self, const python::Vector3Arg& p){ self.setTranslation(p.value); })
        .def("setTranslation",
             [](SgPosTransform& self, const python::Vector3Arg& p){ self.setTranslation(p.value); })
        .def_prop_rw(
            "rotation",
            [](SgPosTransform& self){ return python::eigenMatrixView(self.rotation(), nb::find(&self)); },
            [](SgPosTransform& self, const python::Matrix3RMArg& R){ self.rotation() = R.value; })
        .def("setRotation",
             [](SgPosTransform& self, const python::Matrix3RMArg& R){ self.rotation() = R.value; })
        ;

    nb::class_<SgPreprocessed, SgNode>(m, "SgPreprocessed")
        ;

    nb::class_<SgCamera, SgPreprocessed>(m, "SgCamera")
        .def_static(
            "positionLookingFor",
            [](const python::Vector3Arg& eye, const python::Vector3Arg& direction, const python::Vector3Arg& up) -> Isometry3::MatrixType {
                return SgCamera::positionLookingFor(eye.value, direction.value, up.value).matrix();
            })
        .def_static(
            "positionLookingAt",
            [](const python::Vector3Arg& eye, const python::Vector3Arg& center, const python::Vector3Arg& up) -> Isometry3::MatrixType {
                return SgCamera::positionLookingAt(eye.value, center.value, up.value).matrix();
            })
        .def_static("getRight", [](const python::Matrix4RMArg& T){ return SgCamera::right(Isometry3(T.value)); })
        .def_static("getDirection", [](const python::Matrix4RMArg& T){ return SgCamera::direction(Isometry3(T.value)); })
        .def_static("getUp", [](const python::Matrix4RMArg& T){ return SgCamera::up(Isometry3(T.value)); })
        .def_prop_rw("nearClipDistance", &SgCamera::nearClipDistance, &SgCamera::setNearClipDistance)
        .def("setNearClipDistance", &SgCamera::setNearClipDistance)
        .def_prop_rw("farClipDistance", &SgCamera::farClipDistance, &SgCamera::setFarClipDistance)
        .def("setFarClipDistance", &SgCamera::setFarClipDistance)
        ;

    nb::class_<SgPerspectiveCamera, SgCamera>(m, "SgPerspectiveCamera")
        .def(nb::init<>())
        .def_prop_rw("fieldOfView", &SgPerspectiveCamera::fieldOfView, &SgPerspectiveCamera::setFieldOfView)
        .def("setFieldOfView", &SgPerspectiveCamera::setFieldOfView)
        .def("getFovy", [](SgPerspectiveCamera& self, double aspectRatio){ return self.fovy(aspectRatio); })
        ;

    nb::class_<SgOrthographicCamera, SgCamera>(m, "SgOrthographicCamera")
        .def(nb::init<>())
        .def_prop_rw("height", &SgOrthographicCamera::height, &SgOrthographicCamera::setHeight)
        .def("setHeight", &SgOrthographicCamera::setHeight)
        ;

    nb::class_<SgLight, SgPreprocessed>(m, "SgLight")
        .def_prop_rw("on", (bool(SgLight::*)()const) &SgLight::on, (void(SgLight::*)(bool)) &SgLight::on)
        .def_prop_rw("color",
                     [](SgLight& self){ return self.color(); },
                     [](SgLight& self, const python::Vector3fArg& c){ self.setColor(c.value); })
        .def("setColor", [](SgLight& self, const python::Vector3fArg& c){ self.setColor(c.value); })
        .def_prop_rw("intensity", &SgLight::intensity, &SgLight::setIntensity)
        .def("setIntensity", &SgLight::setIntensity)
        .def_prop_rw("ambientIntensity", &SgLight::ambientIntensity, &SgLight::setAmbientIntensity)
        .def("setAmbientIntensity", &SgLight::setAmbientIntensity)
        ;

    nb::class_<SgDirectionalLight, SgLight>(m, "SgDirectionalLight")
        .def(nb::init<>())
        .def_prop_rw("direction",
                     [](SgDirectionalLight& self){ return self.direction(); },
                     [](SgDirectionalLight& self, const python::Vector3Arg& d){ self.setDirection(d.value); })
        .def("setDirection", [](SgDirectionalLight& self, const python::Vector3Arg& d){ self.setDirection(d.value); })
        ;

    nb::class_<SgPointLight, SgLight>(m, "SgPointLight")
        .def(nb::init<>())
        .def_prop_rw("constantAttenuation", &SgPointLight::constantAttenuation, &SgPointLight::setConstantAttenuation)
        .def("setConstantAttenuation", &SgPointLight::setConstantAttenuation)
        .def_prop_rw("linearAttenuation", &SgPointLight::linearAttenuation, &SgPointLight::setLinearAttenuation)
        .def("setLinearAttenuation", &SgPointLight::setLinearAttenuation)
        .def_prop_rw("quadraticAttenuation", &SgPointLight::quadraticAttenuation, &SgPointLight::setQuadraticAttenuation)
        .def("setQuadraticAttenuation", &SgPointLight::setQuadraticAttenuation)
        ;

    nb::class_<SgSpotLight, SgPointLight>(m, "SgSpotLight")
        .def(nb::init<>())
        .def_prop_rw("direction",
                     [](SgSpotLight& self){ return self.direction(); },
                     [](SgSpotLight& self, const python::Vector3Arg& d){ self.setDirection(d.value); })
        .def("setDirection", [](SgSpotLight& self, const python::Vector3Arg& d){ self.setDirection(d.value); })
        .def_prop_rw("beamWidth", &SgSpotLight::beamWidth, &SgSpotLight::setBeamWidth)
        .def("setBeamWidth", &SgSpotLight::setBeamWidth)
        .def_prop_rw("cutOffAngle", &SgSpotLight::cutOffAngle, &SgSpotLight::setCutOffAngle)
        .def("setCutOffAngle", &SgSpotLight::setCutOffAngle)
        .def_prop_rw("cutOffExponent", &SgSpotLight::cutOffExponent, &SgSpotLight::setCutOffExponent)
        .def("setCutOffExponent", &SgSpotLight::setCutOffExponent)
        ;
}

}
