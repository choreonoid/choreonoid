/*!
  @author Shin'ichiro Nakaoka
*/

#include "PyReferenced.h"
#include "PyEigenTypes.h"
#include "../SceneGraph.h"
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
        .def("getAction", [](SgUpdate& self){ return self.action(); })
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
        .def("isGroupNode", &SgNode::isGroupNode);
    
    py::class_<SgGroup, SgGroupPtr, SgNode>(m, "SgGroup")
        .def(py::init<>())
        .def(py::init<const SgGroup&>())
        .def_property_readonly("empty", &SgGroup::empty)
        .def_property_readonly("numChildren", &SgGroup::numChildren)
        .def("clearChildren", (void(SgGroup::*)(bool)) &SgGroup::clearChildren)
        .def("clearChildren", [](SgGroup& self){ self.clearChildren(); })
        .def_property_readonly("child", (SgNode*(SgGroup::*)(int)) &SgGroup::child)
        .def("addChild", (void(SgGroup::*)(SgNode*, bool)) &SgGroup::addChild)
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
                      (Isometry3& (SgPosTransform::*)()) &SgPosTransform::position,
                      [](SgPosTransform& self, const Isometry3& T) { self.setPosition(T); })
        .def("setPosition", [](SgPosTransform& self, const Isometry3& T) { self.setPosition(T); })
        .def_property("translation",
                      (Isometry3::TranslationPart (SgPosTransform::*)()) &SgPosTransform::translation,
                      [](SgPosTransform& self, const Vector3& p){ self.setTranslation(p); })
        .def("setTranslation", [](SgPosTransform& self, const Vector3& p){ self.setTranslation(p); })
        .def_property("rotation",
                      (Isometry3::LinearPart (SgPosTransform::*)()) &SgPosTransform::rotation,
                      [](SgPosTransform& self, const Matrix3& R) { self.setRotation(R); })
        .def("setRotation", [](SgPosTransform& self, const Matrix3& R) { self.setRotation(R); })
        .def_property("T",
                      (const Isometry3& (SgPosTransform::*)() const ) &SgPosTransform::T,
                      [](SgPosTransform& self, const Isometry3& T) { self.setPosition(T); })

        // deprecated
        .def("getPosition", (Isometry3& (SgPosTransform::*)()) &SgPosTransform::position)
        .def("getTranslation", (Isometry3::TranslationPart (SgPosTransform::*)()) &SgPosTransform::translation)
        .def("getRotation", (Isometry3::LinearPart (SgPosTransform::*)()) &SgPosTransform::rotation)
        ;
}

}
