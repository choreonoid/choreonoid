/*!
  @author Shin'ichiro Nakaoka
*/

#include "PyUtil.h"
#include "../SceneGraph.h"
#include "../SceneProvider.h"

using namespace boost::python;
using namespace cnoid;

namespace {

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(SgGroup_clearChildren, clearChildren, 0, 1)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(SgGroup_addChild, addChild, 1, 2)

SgNodePtr SgGroup_child(SgGroup& self, int index) { return self.child(index); }

SgNodePtr SceneProvider_getScene(SceneProvider& self) { return self.getScene(); }

}

namespace cnoid {

void exportPySceneGraph()
{
    class_< SgObject, SgObjectPtr, bases<Referenced>, boost::noncopyable >("SgObject", no_init)
        .def("name", &SgObject::name, return_value_policy<return_by_value>())
        .def("setName", &SgObject::setName);

    implicitly_convertible<SgObjectPtr, ReferencedPtr>();

    class_< SgNode, SgNodePtr, bases<SgObject> >("SgNode")
        .def("isGroup", &SgNode::isGroup);

    implicitly_convertible<SgNodePtr, SgObjectPtr>();
    
    class_< SgGroup, SgGroupPtr, bases<SgNode> >("SgGroup")
        .def("empty", &SgGroup::empty)
        .def("numChildren", &SgGroup::numChildren)
        .def("clearChildren", &SgGroup::clearChildren, SgGroup_clearChildren())
        .def("child", SgGroup_child)
        .def("addChild", &SgGroup::addChild, SgGroup_addChild());

    implicitly_convertible<SgGroupPtr, SgNodePtr>();
    
    class_<SceneProvider, boost::noncopyable>("SceneProvider", no_init)
        .def("getScene", SceneProvider_getScene);
}

}

