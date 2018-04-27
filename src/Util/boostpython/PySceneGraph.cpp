/*!
  @author Shin'ichiro Nakaoka
*/

#include "PyUtil.h"
#include "../SceneGraph.h"
#include "../SceneProvider.h"

using namespace boost::python;
using namespace cnoid;

namespace {

void SgObject_notifyUpdate1(SgObject& self){ self.notifyUpdate(); }
void SgObject_notifyUpdate2(SgObject& self, SgUpdate& update){ self.notifyUpdate(update); }
void SgObject_notifyUpdate3(SgObject& self, SgUpdate::Action action){ self.notifyUpdate(action); }

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(SgGroup_clearChildren, clearChildren, 0, 1)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(SgGroup_addChild, addChild, 1, 2)

SgNodePtr SgGroup_child(SgGroup& self, int index) { return self.child(index); }

SgNodePtr SceneProvider_getScene(SceneProvider& self) { return self.getScene(); }

Affine3 SgPosTransform_get_position(SgPosTransform& self) { return self.position(); }
void SgPosTransform_set_position(SgPosTransform& self, const Affine3& T) { self.position() = T; }
Affine3 SgPosTransform_get_T(SgPosTransform& self) { return self.T(); }
void SgPosTransform_set_T(SgPosTransform& self, const Affine3& T) { self.T() = T; }
Vector3 SgPosTransform_get_translation(SgPosTransform& self) { return self.translation(); }
void SgPosTransform_set_translation(SgPosTransform& self, const Vector3& p) { self.setTranslation(p); }
Matrix3 SgPosTransform_get_rotation(SgPosTransform& self) { return self.rotation(); }
void SgPosTransform_set_rotation(SgPosTransform& self, const Matrix3& R){ self.setRotation(R); }

}

namespace cnoid {

void exportPySceneGraph()
{
    {
        scope sgObjectScope = 
            class_< SgUpdate >("SgUpdate")
            .def("action", &SgUpdate::action)
            .def("getAction", &SgUpdate::action)
            .def("setAction", &SgUpdate::setAction)
            ;

        enum_<SgUpdate::Action>("Action")
            .value("NONE", SgUpdate::NONE)
            .value("ADDED", SgUpdate::ADDED)
            .value("REMOVED", SgUpdate::REMOVED)
            .value("BBOX_UPDATED", SgUpdate::BBOX_UPDATED)
            .value("MODIFIED", SgUpdate::MODIFIED)
            ;
    }
        
    class_< SgObject, SgObjectPtr, bases<Referenced>, boost::noncopyable >("SgObject", no_init)
        .def("name", &SgObject::name, return_value_policy<return_by_value>())
        .def("getName", &SgObject::name, return_value_policy<return_by_value>())
        .def("setName", &SgObject::setName)
        .def("notifyUpdate", SgObject_notifyUpdate1)
        .def("notifyUpdate", SgObject_notifyUpdate2)
        .def("notifyUpdate", SgObject_notifyUpdate3)
        ;
        
    implicitly_convertible<SgObjectPtr, ReferencedPtr>();

    class_< SgNode, SgNodePtr, bases<SgObject> >("SgNode")
        .def("isGroup", &SgNode::isGroup);

    implicitly_convertible<SgNodePtr, SgObjectPtr>();
    
    class_< SgGroup, SgGroupPtr, bases<SgNode> >("SgGroup")
        .def("empty", &SgGroup::empty)
        .def("isEmpty", &SgGroup::empty)
        .def("numChildren", &SgGroup::numChildren)
        .def("getNumChildren", &SgGroup::numChildren)
        .def("clearChildren", &SgGroup::clearChildren, SgGroup_clearChildren())
        .def("child", SgGroup_child)
        .def("getChild", SgGroup_child)
        .def("addChild", &SgGroup::addChild, SgGroup_addChild());

    implicitly_convertible<SgGroupPtr, SgNodePtr>();
    
    class_< SgTransform, SgTransformPtr, bases<SgGroup>, boost::noncopyable>("SgTransform", no_init);

    implicitly_convertible<SgTransformPtr, SgGroupPtr>();

    class_< SgPosTransform, SgPosTransformPtr, bases<SgTransform>, boost::noncopyable >("SgPosTransform")
        .def("position", SgPosTransform_get_position)
        .def("getPosition", SgPosTransform_get_position)
        .def("setPosition", SgPosTransform_set_position)
        .add_property("T", SgPosTransform_get_position, SgPosTransform_set_position)
        .def("translation", SgPosTransform_get_translation)
        .def("getTranslation", SgPosTransform_get_translation)
        .def("setTranslation", SgPosTransform_set_translation)        
        .def("rotation", SgPosTransform_get_rotation)
        .def("getRotation", SgPosTransform_get_rotation)
        .def("setRotation", SgPosTransform_set_rotation)
        ;
    implicitly_convertible<SgPosTransformPtr, SgTransformPtr>();

    class_<SceneProvider, SceneProvider*, boost::noncopyable>("SceneProvider", no_init);
}

}
