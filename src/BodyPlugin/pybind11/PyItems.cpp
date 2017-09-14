/*!
  @author Shin'ichiro Nakaoka
*/

#include "../BodyMotionItem.h"
#include "../WorldItem.h"
#include <cnoid/PyBase>

using namespace cnoid;
namespace py = pybind11;

namespace cnoid {

void exportItems(py::module m)
{
    py::class_<WorldItem, WorldItemPtr, Item>(m, "WorldItem", py::multiple_inheritance())
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
    
    py::class_<BodyMotionItem, BodyMotionItemPtr, AbstractMultiSeqItem>(m, "BodyMotionItem")
        .def(py::init<>())
        .def("motion", (BodyMotionPtr(BodyMotionItem::*)()) &BodyMotionItem::motion)
        .def("jointPosSeqItem", (MultiValueSeqItem*(BodyMotionItem::*)())&BodyMotionItem::jointPosSeqItem)
        .def("jointPosSeq", &BodyMotionItem::jointPosSeq)
        .def("linkPosSeqItem", (MultiSE3SeqItem*(BodyMotionItem::*)())&BodyMotionItem::linkPosSeqItem)
        .def("linkPosSeq", &BodyMotionItem::linkPosSeq)
        .def("numExtraSeqItems", &BodyMotionItem::numExtraSeqItems)
        .def("extraSeqKey", &BodyMotionItem::extraSeqKey)
        .def("extraSeqItem", (AbstractSeqItem*(BodyMotionItem::*)(int))&BodyMotionItem::extraSeqItem)
        .def("updateExtraSeqItems", &BodyMotionItem::updateExtraSeqItems)
        ;

    PyItemList<BodyMotionItem>(m, "BodyMotionItemList");
}

}
