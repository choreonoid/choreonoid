/*!
  @author Shin'ichiro Nakaoka
*/

#include "../BodyMotionItem.h"
#include "../WorldItem.h"
#include <cnoid/Py3Base>

namespace py = pybind11;
using namespace cnoid;

namespace {

BodyMotionPtr BodyMotionItem_motion(BodyMotionItem& self) { return self.motion(); }
MultiValueSeqItemPtr BodyMotionItem_jointPosSeqItem(BodyMotionItem& self) { return self.jointPosSeqItem(); }
MultiSE3SeqItemPtr BodyMotionItem_linkPosSeqItem(BodyMotionItem& self) { return self.linkPosSeqItem(); }
AbstractSeqItemPtr BodyMotionItem_extraSeqItem(BodyMotionItem& self, int index) { return self.extraSeqItem(index); }

}

void exportItems(py::module m)
{
    py::class_< WorldItem, WorldItemPtr, Item>(m, "WorldItem")
        .def(py::init<>())
        .def("selectCollisionDetector", &WorldItem::selectCollisionDetector)
        .def("enableCollisionDetection", &WorldItem::enableCollisionDetection)
        .def("isCollisionDetectionEnabled", &WorldItem::isCollisionDetectionEnabled)
        .def("updateCollisionDetectorLater", &WorldItem::updateCollisionDetectorLater)
        .def("updateCollisionDetector", &WorldItem::updateCollisionDetector)
        .def("updateCollisions", &WorldItem::updateCollisions)
        .def("sigCollisionsUpdated", &WorldItem::sigCollisionsUpdated)
        ;

    PyItemList<WorldItem>(m, "WorldItemList");
    
    py::class_< BodyMotionItem, BodyMotionItemPtr, AbstractMultiSeqItem>(m, "BodyMotionItem")
        .def(py::init<>())
        .def("motion", [](BodyMotionItem& self){ return BodyMotionPtr(self.motion()); })
        .def("jointPosSeqItem", [](BodyMotionItem& self){ return MultiValueSeqItemPtr(self.jointPosSeqItem()); })
        .def("jointPosSeq", &BodyMotionItem::jointPosSeq)
        .def("linkPosSeqItem", [](BodyMotionItem& self){ return MultiSE3SeqItemPtr(self.linkPosSeqItem()); })
        .def("linkPosSeq", &BodyMotionItem::linkPosSeq)
        .def("numExtraSeqItems", &BodyMotionItem::numExtraSeqItems)
        .def("extraSeqKey", &BodyMotionItem::extraSeqKey, py::return_value_policy::reference)
        .def("extraSeqItem", [](BodyMotionItem& self, int index) { return AbstractSeqItemPtr(self.extraSeqItem(index)); })
        .def("updateExtraSeqItems", &BodyMotionItem::updateExtraSeqItems)
        ;

    PyItemList<BodyMotionItem>(m, "BodyMotionItemList");
}
