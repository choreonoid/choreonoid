/*!
  @author Shin'ichiro Nakaoka
*/

#include "PyUtil.h"
#include "../SceneGraph.h"
#include "../SceneProvider.h"

namespace py = pybind11;
using namespace cnoid;

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

    py::class_<SgObject, SgObjectPtr, Referenced>(m, "SgObject")
        .def("name", &SgObject::name)
        .def("setName", &SgObject::setName)
        .def("notifyUpdate",(void(SgObject::*)(SgUpdate&)) &SgObject::notifyUpdate)
        .def("notifyUpdate",[](SgObject* self){ self->notifyUpdate(); })
        .def("notifyUpdate",[](SgObject* self, int action){ self->notifyUpdate(action); })
        ;

    py::class_<SgNode, SgNodePtr, SgObject>(m, "SgNode")
        .def("isGroup", &SgNode::isGroup);
    
    py::class_<SgGroup, SgGroupPtr, SgNode>(m, "SgGroup")
        .def("empty", &SgGroup::empty)
        .def("numChildren", &SgGroup::numChildren)
        .def("clearChildren", [](SgGroup* self){ self->clearChildren(); })
        .def("clearChildren", [](SgGroup* self, bool doNotify){ self->clearChildren(doNotify); })
        .def("child", [](SgGroup* self, int index){ return SgNodePtr(self->child(index)); })
        .def("addChild", [](SgGroup* self, SgNodePtr node){ self->addChild(node); })
        .def("addChild", [](SgGroup* self, SgNodePtr node, bool doNotify){ self->addChild(node, doNotify); })
        ;
    
    py::class_<SgTransform, SgTransformPtr, SgGroup>(m, "SgTransform");

    py::class_<SgPosTransform, SgPosTransformPtr, SgTransform>(m, "SgPosTransform")
        .def("position", (Affine3& (SgPosTransform::*)()) &SgPosTransform::position)
        .def("setPosition", [](SgPosTransform* self, const Affine3& T) { self->setPosition(T); })
        .def("translation", (Affine3::TranslationPart (SgPosTransform::*)()) &SgPosTransform::translation)
        .def("setTranslation", [](SgPosTransform* self, const Vector3& p){ self->setTranslation(p); })
        .def("rotation", (Affine3::LinearPart (SgPosTransform::*)()) &SgPosTransform::rotation)
        .def("setRotation", [](SgPosTransform* self, const Matrix3& R) { self->setRotation(R); })
        .def_property("T",
                      (const Affine3& (SgPosTransform::*)() const ) &SgPosTransform::T,
                      [](SgPosTransform* self, const Affine3& T) { self->setPosition(T); })
        ;

    py::class_<SceneProvider>(m, "SceneProvider");
}

}
