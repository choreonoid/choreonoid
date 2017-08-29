/*!
  @author Shin'ichiro Nakaoka
*/

#include "PyReferenced.h"
#include "PyEigenTypes.h"
#include "../SceneGraph.h"
#include "../SceneProvider.h"

using namespace cnoid;
namespace py = pybind11;

namespace cnoid {

void exportPySceneGraph(py::module& m)
{
    py::class_<SgUpdate> sgUpdate(m, "SgUpdate");

    sgUpdate
        .def(py::init<>())
        .def(py::init<int>())
        .def("action", &SgUpdate::action)
        .def("setAction", &SgUpdate::setAction)
        ;

    py::enum_<SgUpdate::Action>(sgUpdate, "Action")
        .value("NONE", SgUpdate::Action::NONE)
        .value("ADDED", SgUpdate::Action::ADDED)
        .value("REMOVED", SgUpdate::Action::REMOVED)
        .value("BBOX_UPDATED", SgUpdate::Action::BBOX_UPDATED)
        .value("MODIFIED", SgUpdate::Action::MODIFIED)
        .export_values();

    py::class_<SgCloneMap>(m, "SgCloneMap")
        .def(py::init<>())
        .def(py::init<const SgCloneMap&>())
        .def("setNonNodeCloning", &SgCloneMap::setNonNodeCloning)
        .def("isNonNodeCloningEnabled", &SgCloneMap::isNonNodeCloningEnabled)
        .def("clear", &SgCloneMap::clear)
        ;

    py::class_<SgObject, SgObjectPtr, Referenced>(m, "SgObject")
        .def("name", &SgObject::name)
        .def("setName", &SgObject::setName)
        .def("notifyUpdate",(void(SgObject::*)(SgUpdate&)) &SgObject::notifyUpdate)
        .def("notifyUpdate",(void(SgObject::*)(int)) &SgObject::notifyUpdate)
        .def("notifyUpdate",[](SgObject& self){ self.notifyUpdate(); })
        ;

    py::class_<SgNode, SgNodePtr, SgObject>(m, "SgNode")
        .def(py::init<>())
        .def(py::init<const SgNode&>())
        .def("isGroup", &SgNode::isGroup);
    
    py::class_<SgGroup, SgGroupPtr, SgNode>(m, "SgGroup")
        .def(py::init<>())
        .def(py::init<const SgGroup&>())
        .def("empty", &SgGroup::empty)
        .def("numChildren", &SgGroup::numChildren)
        .def("clearChildren", &SgGroup::clearChildren)
        .def("clearChildren", [](SgGroup& self){ self.clearChildren(); })
        .def("child", (SgNode*(SgGroup::*)(int)) &SgGroup::child)
        .def("addChild", &SgGroup::addChild)
        .def("addChild", [](SgGroup& self, SgNode* node){ self.addChild(node); })
        ;
    
    py::class_<SgTransform, SgTransformPtr, SgGroup>(m, "SgTransform");

    py::class_<SgPosTransform, SgPosTransformPtr, SgTransform>(m, "SgPosTransform")
        .def(py::init<>())
        .def(py::init<const SgPosTransform&>())
        .def("position", (Affine3& (SgPosTransform::*)()) &SgPosTransform::position)
        .def("setPosition", [](SgPosTransform& self, const Affine3& T) { self.setPosition(T); })
        .def("translation", (Affine3::TranslationPart (SgPosTransform::*)()) &SgPosTransform::translation)
        .def("setTranslation", [](SgPosTransform& self, const Vector3& p){ self.setTranslation(p); })
        .def("rotation", (Affine3::LinearPart (SgPosTransform::*)()) &SgPosTransform::rotation)
        .def("setRotation", [](SgPosTransform& self, const Matrix3& R) { self.setRotation(R); })
        .def_property("T",
                      (const Affine3& (SgPosTransform::*)() const ) &SgPosTransform::T,
                      [](SgPosTransform& self, const Affine3& T) { self.setPosition(T); })
        ;

    py::class_<SceneProvider>(m, "SceneProvider")
        .def("getScene", (SgNode*(SceneProvider::*)()) &SceneProvider::getScene)
        .def("getScene", (SgNode*(SceneProvider::*)(SgCloneMap&)) &SceneProvider::getScene)
        ;
}

}
