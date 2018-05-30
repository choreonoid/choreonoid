/*!
  @author Shin'ichiro Nakaoka
*/

#include "../BodyMotionItem.h"
#include "../WorldItem.h"
#include <cnoid/PyBase>

using namespace boost::python;
using namespace cnoid;

namespace {

BodyMotionPtr BodyMotionItem_motion(BodyMotionItem& self) { return self.motion(); }
MultiValueSeqItemPtr BodyMotionItem_jointPosSeqItem(BodyMotionItem& self) { return self.jointPosSeqItem(); }
MultiSE3SeqItemPtr BodyMotionItem_linkPosSeqItem(BodyMotionItem& self) { return self.linkPosSeqItem(); }
AbstractSeqItemPtr BodyMotionItem_extraSeqItem(BodyMotionItem& self, int index) { return self.extraSeqItem(index); }

}

void exportItems()
{
    class_<WorldItem, WorldItemPtr, bases<Item, SceneProvider>>("WorldItem")
        .def("selectCollisionDetector", &WorldItem::selectCollisionDetector)
        .def("enableCollisionDetection", &WorldItem::enableCollisionDetection)
        .def("isCollisionDetectionEnabled", &WorldItem::isCollisionDetectionEnabled)
        .def("updateCollisionDetectorLater", &WorldItem::updateCollisionDetectorLater)
        .def("updateCollisionDetector", &WorldItem::updateCollisionDetector)
        .def("updateCollisions", &WorldItem::updateCollisions)
        .def("sigCollisionsUpdated", &WorldItem::sigCollisionsUpdated)
        .def("getSigCollisionsUpdated", &WorldItem::sigCollisionsUpdated)
        ;

    implicitly_convertible<WorldItemPtr, ItemPtr>();
    implicitly_convertible<WorldItemPtr, SceneProvider*>();
    PyItemList<WorldItem>("WorldItemList");
    
    class_<BodyMotionItem, BodyMotionItemPtr, bases<AbstractSeqItem>>("BodyMotionItem")
        .def("motion", BodyMotionItem_motion)
        .def("getMotion", BodyMotionItem_motion)
        .def("jointPosSeqItem", BodyMotionItem_jointPosSeqItem)
        .def("getJointPosSeqItem", BodyMotionItem_jointPosSeqItem)
        .def("jointPosSeq", &BodyMotionItem::jointPosSeq)
        .def("getJointPosSeq", &BodyMotionItem::jointPosSeq)
        .def("linkPosSeqItem", BodyMotionItem_linkPosSeqItem)
        .def("getLinkPosSeqItem", BodyMotionItem_linkPosSeqItem)
        .def("linkPosSeq", &BodyMotionItem::linkPosSeq)
        .def("getLinkPosSeq", &BodyMotionItem::linkPosSeq)
        .def("numExtraSeqItems", &BodyMotionItem::numExtraSeqItems, return_value_policy<return_by_value>())
        .def("getNumExtraSeqItems", &BodyMotionItem::numExtraSeqItems, return_value_policy<return_by_value>())
        .def("extraSeqKey", &BodyMotionItem::extraSeqKey, return_value_policy<copy_const_reference>())
        .def("extraSeqItem", BodyMotionItem_extraSeqItem)
        .def("updateExtraSeqItems", &BodyMotionItem::updateExtraSeqItems)
        ;

    implicitly_convertible<BodyMotionItemPtr, AbstractSeqItemPtr>();
    PyItemList<BodyMotionItem>("BodyMotionItemList");
}
