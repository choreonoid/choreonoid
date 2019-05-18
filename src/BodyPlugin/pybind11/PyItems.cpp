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
        .def_property_readonly("sigCollisionsUpdated", &WorldItem::sigCollisionsUpdated)

        // deprecated
        .def("getSigCollisionsUpdated", &WorldItem::sigCollisionsUpdated)
        ;

    PyItemList<WorldItem>(m, "WorldItemList");
    
    py::class_<BodyMotionItem, BodyMotionItemPtr, AbstractSeqItem>(m, "BodyMotionItem")
        .def(py::init<>())
        .def_property_readonly("motion", [](BodyMotionItem* item){ return item->motion(); })
        .def_property_readonly("jointPosSeqItem", (MultiValueSeqItem*(BodyMotionItem::*)())&BodyMotionItem::jointPosSeqItem)
        .def_property_readonly("jointPosSeq", &BodyMotionItem::jointPosSeq)
        .def_property_readonly("linkPosSeqItem", (MultiSE3SeqItem*(BodyMotionItem::*)())&BodyMotionItem::linkPosSeqItem)
        .def_property_readonly("linkPosSeq", &BodyMotionItem::linkPosSeq)
        .def_property_readonly("numExtraSeqItems", &BodyMotionItem::numExtraSeqItems)
        .def("extraSeqKey", &BodyMotionItem::extraSeqKey)
        .def("extraSeqItem", (AbstractSeqItem*(BodyMotionItem::*)(int))&BodyMotionItem::extraSeqItem)
        .def("updateExtraSeqItems", &BodyMotionItem::updateExtraSeqItems)

        // deprecated
        .def("getMotion", [](BodyMotionItem* item){ return item->motion(); })
        .def("getJointPosSeqItem", (MultiValueSeqItem*(BodyMotionItem::*)())&BodyMotionItem::jointPosSeqItem)
        .def("getJointPosSeq", &BodyMotionItem::jointPosSeq)
        .def("getLinkPosSeqItem", (MultiSE3SeqItem*(BodyMotionItem::*)())&BodyMotionItem::linkPosSeqItem)
        .def("getLinkPosSeq", &BodyMotionItem::linkPosSeq)
        .def("getNumExtraSeqItems", &BodyMotionItem::numExtraSeqItems)
        ;

    PyItemList<BodyMotionItem>(m, "BodyMotionItemList");
}

}
