/*!
  @author Shin'ichiro Nakaoka
*/

#include "PyReferenced.h"
#include "PyEigenTypes.h"
#include "../SceneGraph.h"
#include "../SceneProvider.h"
#include "../CloneMap.h"

using namespace cnoid;
namespace py = pybind11;

namespace cnoid {

void exportPySceneGraph(py::module& m)
{
    py::class_<SgUpdate> sgUpdate(m, "SgUpdate");

    sgUpdate
        .def(py::init<>())
        .def(py::init<int>())
        .def_property("action", &SgUpdate::action, &SgUpdate::setAction)
        .def("setAction", &SgUpdate::setAction)

        // deprecated
        .def("getAction", &SgUpdate::action)
        ;

    py::enum_<SgUpdate::Action>(sgUpdate, "Action")
        .value("NONE", SgUpdate::Action::NONE)
        .value("ADDED", SgUpdate::Action::ADDED)
        .value("REMOVED", SgUpdate::Action::REMOVED)
        .value("BBOX_UPDATED", SgUpdate::Action::BBOX_UPDATED)
        .value("MODIFIED", SgUpdate::Action::MODIFIED)
        .export_values();

    py::class_<SgObject, SgObjectPtr, Referenced>(m, "SgObject")
        .def_property("name", &SgObject::name, &SgObject::setName)
        .def("setName", &SgObject::setName)
        .def("notifyUpdate",(void(SgObject::*)(SgUpdate&)) &SgObject::notifyUpdate)
        .def("notifyUpdate",(void(SgObject::*)(int)) &SgObject::notifyUpdate)
        .def("notifyUpdate",[](SgObject& self){ self.notifyUpdate(); })

        // deprecated
        .def("getName", &SgObject::name)
        ;

    py::class_<SgNode, SgNodePtr, SgObject>(m, "SgNode")
        .def(py::init<>())
        .def(py::init<const SgNode&>())
        .def("isGroup", &SgNode::isGroup);
    
    py::class_<SgGroup, SgGroupPtr, SgNode>(m, "SgGroup")
        .def(py::init<>())
        .def(py::init<const SgGroup&>())
        .def_property_readonly("empty", &SgGroup::empty)
        .def_property_readonly("numChildren", &SgGroup::numChildren)
        .def("clearChildren", &SgGroup::clearChildren)
        .def("clearChildren", [](SgGroup& self){ self.clearChildren(); })
        .def_property_readonly("child", (SgNode*(SgGroup::*)(int)) &SgGroup::child)
        .def("addChild", &SgGroup::addChild)
        .def("addChild", [](SgGroup& self, SgNode* node){ self.addChild(node); })

        // deprecated
        .def("isEmpty", &SgGroup::empty)
        .def("getNumChildren", &SgGroup::numChildren)
        .def("getChild", (SgNode*(SgGroup::*)(int)) &SgGroup::child)
        ;
    
    py::class_<SgTransform, SgTransformPtr, SgGroup>(m, "SgTransform");

    py::class_<SgPosTransform, SgPosTransformPtr, SgTransform>(m, "SgPosTransform")
        .def(py::init<>())
        .def(py::init<const SgPosTransform&>())
        .def_property("position",
                      (Affine3& (SgPosTransform::*)()) &SgPosTransform::position,
                      [](SgPosTransform& self, const Affine3& T) { self.setPosition(T); })
        .def("setPosition", [](SgPosTransform& self, const Affine3& T) { self.setPosition(T); })
        .def_property("translation",
                      (Affine3::TranslationPart (SgPosTransform::*)()) &SgPosTransform::translation,
                      [](SgPosTransform& self, const Vector3& p){ self.setTranslation(p); })
        .def("setTranslation", [](SgPosTransform& self, const Vector3& p){ self.setTranslation(p); })
        .def_property("rotation",
                      (Affine3::LinearPart (SgPosTransform::*)()) &SgPosTransform::rotation,
                      [](SgPosTransform& self, const Matrix3& R) { self.setRotation(R); })
        .def("setRotation", [](SgPosTransform& self, const Matrix3& R) { self.setRotation(R); })
        .def_property("T",
                      (const Affine3& (SgPosTransform::*)() const ) &SgPosTransform::T,
                      [](SgPosTransform& self, const Affine3& T) { self.setPosition(T); })

        // deprecated
        .def("getPosition", (Affine3& (SgPosTransform::*)()) &SgPosTransform::position)
        .def("getTranslation", (Affine3::TranslationPart (SgPosTransform::*)()) &SgPosTransform::translation)
        .def("getRotation", (Affine3::LinearPart (SgPosTransform::*)()) &SgPosTransform::rotation)
        ;

    py::class_<SceneProvider>(m, "SceneProvider")
        .def("getScene", (SgNode*(SceneProvider::*)()) &SceneProvider::getScene)
        .def("getScene", (SgNode*(SceneProvider::*)(CloneMap&)) &SceneProvider::getScene)
        ;
}

}
