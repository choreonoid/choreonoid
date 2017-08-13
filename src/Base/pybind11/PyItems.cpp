/*!
  @author Shin'ichiro Nakaoka
*/

#include "PyBase.h"
#include "../Item.h"
#include "../RootItem.h"
#include "../FolderItem.h"
#include "../AbstractTextItem.h"
#include "../ScriptItem.h"
#include "../ExtCommandItem.h"
#include "../MultiValueSeqItem.h"
#include "../MultiAffine3SeqItem.h"
#include "../MultiSE3SeqItem.h"
#include "../Vector3SeqItem.h"
#include "../SceneItem.h"
#include "../PointSetItem.h"
#include "../MultiPointSetItem.h"
#include <cnoid/PyUtil>

namespace py = pybind11;
using namespace cnoid;

namespace cnoid {

void exportPyItems(py::module m)
{
    py::class_<Item, ItemPtr> itemClass(m, "Item");

    itemClass
        .def_static("find", [](const std::string& path){
            return ItemPtr(Item::find(path));
        })
        .def("name", &Item::name, py::return_value_policy::reference)
        .def("setName", &Item::setName)
        .def("hasAttribute", &Item::hasAttribute)
        .def("childItem", [](Item& self) { return ItemPtr(self.childItem()); })
        .def("prevItem", [](Item& self) { return ItemPtr(self.prevItem()); })
        .def("nextItem", [](Item& self) { return ItemPtr(self.nextItem()); })
        .def("parentItem", [](Item& self) { return ItemPtr(self.parentItem()); })
        .def("addChildItem", &Item::addChildItem, py::arg("item"), py::arg("isManualOperation")=false)
        .def("addSubItem", &Item::addSubItem)
        .def("isSubItem", &Item::isSubItem)
        .def("detachFromParentItem", &Item::detachFromParentItem)
        .def("emitSigDetachedFromRootForSubTree", &Item::emitSigDetachedFromRootForSubTree)
        .def("insertChildItem", &Item::insertChildItem, py::arg("item"), py::arg("nextItem"),
                py::arg("isManualOperation")=false)
        .def("insertSubItem", &Item::insertSubItem)
        .def("isTemporal", &Item::isTemporal)
        .def("setTemporal", &Item::setTemporal, py::arg("on")=true)
        .def("findRootItem", [](Item& self) { return RootItemPtr(self.findRootItem()); })
        .def("findItem", [](Item& self, const std::string& path) { return ItemPtr(self.findItem(path)); })
        .def("findChildItem", [](Item& self, const std::string& path) { return ItemPtr(self.findChildItem(path)); })
        .def("findSubItem", [](Item& self, const std::string& path) { return ItemPtr(self.findSubItem(path)); })
        .def("headItem", [](Item& self) { return ItemPtr(self.headItem()); })
        .def("getDescendantItems", [](Item& self) {
            ItemList<Item> items;
            items.extractChildItems(&self);
            return items;
        })
        .def("getDescendantItems", [](Item& self, py::object itemClass) {
            ItemList<Item> items;
            items.extractChildItems(&self);
            return getPyNarrowedItemList(items, itemClass);
        })
        .def("duplicate", [](Item& self) { return ItemPtr(self.duplicate()); })
        .def("duplicateAll", [](Item& self) { return ItemPtr(self.duplicateAll()); })
        .def("assign", &Item::assign)
        .def("load", (bool (Item::*)(const std::string&, const std::string&)) &Item::load,
                py::arg("filename"), py::arg("format")=std::string() )
        .def("load", (bool (Item::*)(const std::string&, Item*, const std::string&)) &Item::load,
                py::arg("filename"), py::arg("parent"), py::arg("format")=std::string() )
        .def("save", &Item::save, py::arg("filename"), py::arg("format")=std::string() )
        .def("overwrite", &Item::overwrite,
                py::arg("forceOverwrite")=false, py::arg("format")=std::string() )
        .def("filePath", &Item::filePath, py::return_value_policy::reference)
        .def("fileFormat", &Item::fileFormat, py::return_value_policy::reference)
        .def("clearFileInformation", &Item::clearFileInformation)
        .def("suggestFileUpdate", &Item::suggestFileUpdate)
        .def("notifyUpdate", &Item::notifyUpdate)
        .def("sigNameChanged", &Item::sigNameChanged)
        .def("sigUpdated", &Item::sigUpdated)
        .def("sigPositionChanged", &Item::sigPositionChanged)
        .def("sigDisconnectedFromRoot", &Item::sigDisconnectedFromRoot)
        .def("sigSubTreeChanged", &Item::sigSubTreeChanged);

    py::enum_<Item::Attribute>(itemClass, "Attribute")
        .value("SUB_ITEM", Item::Attribute::SUB_ITEM)
        .value("TEMPORAL", Item::Attribute::TEMPORAL)
        .value("LOAD_ONLY", Item::Attribute::LOAD_ONLY)
        .value("NUM_ATTRIBUTES", Item::Attribute::NUM_ATTRIBUTES)
        .export_values();

    PyItemList<Item>(m, "ItemList");

    py::class_< RootItem, RootItemPtr, Item >(m, "RootItem")
        .def_static("instance", []() { return RootItemPtr(RootItem::instance()); });

    PyItemList<RootItem>(m, "RootItemList");

    py::class_< FolderItem, FolderItemPtr, Item >(m, "FolderItem")
        .def(py::init<>());

    PyItemList<FolderItem>(m, "FolderItemList");

    py::class_< AbstractTextItem, AbstractTextItemPtr, Item >(m, "AbstractTextItem")
        .def("textFilename", &AbstractTextItem::textFilename, py::return_value_policy::reference);
            
    //PyItemList<AbstractTextItem>("AbstractTextItemList");
    
    py::class_< ScriptItem, ScriptItemPtr, AbstractTextItem> (m, "ScriptItem")
        .def("scriptFilename", &ScriptItem::scriptFilename, py::return_value_policy::reference)
        .def("identityName", &ScriptItem::identityName)
        .def("setBackgroundMode", &ScriptItem::setBackgroundMode)
        .def("isBackgroundMode", &ScriptItem::isBackgroundMode)
        .def("isRunning", &ScriptItem::isRunning)
        .def("execute", &ScriptItem::execute)
        .def("waitToFinish", &ScriptItem::waitToFinish, py::arg("timeout")=0.0)
        .def("resultString", &ScriptItem::resultString)
        .def("sigScriptFinished", &ScriptItem::sigScriptFinished)
        .def("terminate", &ScriptItem::terminate)
        ;

    //PyItemList<ScriptItem>("ScriptItemList");

    py::class_< ExtCommandItem, ExtCommandItemPtr, Item >(m, "ExtCommandItem")
        .def(py::init<>())
        .def("setCommand", &ExtCommandItem::setCommand)
        .def("command", &ExtCommandItem::command, py::return_value_policy::reference)
        .def("waitingTimeAfterStarted", &ExtCommandItem::waitingTimeAfterStarted)
        .def("setWaitingTimeAfterStarted", &ExtCommandItem::setWaitingTimeAfterStarted)
        .def("execute", &ExtCommandItem::execute)
        .def("terminate", &ExtCommandItem::terminate);
    
    PyItemList<ExtCommandItem>(m, "ExtCommandItemList");

    // seq items
    py::class_< AbstractSeqItem, AbstractSeqItemPtr, Item > abstractSeqItemClass(m, "AbstractSeqItem");
    abstractSeqItemClass
        .def("abstractSeq", &AbstractSeqItem::abstractSeq);

    PyItemList<AbstractSeqItem>(m, "AbstractSeqItemList", abstractSeqItemClass);

    py::class_< Vector3SeqItem, Vector3SeqItemPtr, AbstractSeqItem >(m, "Vector3SeqItem")
        .def(py::init<>())
        .def("seq", &Vector3SeqItem::seq);

    PyItemList<Vector3SeqItem>(m, "Vector3SeqItemList");

    // multi seq items
    py::class_< AbstractMultiSeqItem, AbstractMultiSeqItemPtr, AbstractSeqItem >
        abstractMultiSeqItemClass(m, "AbstractMultiSeqItem");
    abstractMultiSeqItemClass
        .def("abstractMultiSeq", &AbstractMultiSeqItem::abstractMultiSeq);

    //PyItemList<AbstractMultiSeqItem>("AbstractMultiSeqItemList", abstractMultiSeqItemClass);
    
    py::class_< MultiValueSeqItem, MultiValueSeqItemPtr, AbstractMultiSeqItem >(m, "MultiValueSeqItem")
        .def(py::init<>())
        .def("abstractMultiSeq", &MultiValueSeqItem::abstractMultiSeq)
        .def("seq", &MultiValueSeqItem::seq);

    PyItemList<MultiValueSeqItem>(m, "MultiValueSeqItemList");

    py::class_< MultiAffine3SeqItem, MultiAffine3SeqItemPtr, AbstractMultiSeqItem >(m, "MultiAffine3SeqItem")
        .def(py::init<>())
        .def("abstractMultiSeq", &MultiAffine3SeqItem::abstractMultiSeq)
        .def("seq", &MultiAffine3SeqItem::seq);

    PyItemList<MultiAffine3SeqItem>(m, "MultiAffine3SeqItemList");

    py::class_< MultiSE3SeqItem, MultiSE3SeqItemPtr, AbstractMultiSeqItem > (m, "MultiSE3SeqItem")
        .def(py::init<>())
        .def("abstractMultiSeq", &MultiSE3SeqItem::abstractMultiSeq)
        .def("seq", &MultiSE3SeqItem::seq);
    
    PyItemList<MultiSE3SeqItem>(m, "MultiSE3SeqItemList");

    py::class_< SceneItem, SceneItemPtr, Item>(m, "SceneItem")
        .def(py::init<>())
        .def("topNode", [](SceneItem& self) { return SgPosTransformPtr(self.topNode()); })
        ;

    py::class_< PointSetItem, PointSetItemPtr, Item > (m, "PointSetItem")
        .def(py::init<>())
        .def("offsetTransform", [](PointSetItem& self) {
            return Affine3(self.offsetTransform());
        })
        .def("setOffsetTransform", &PointSetItem::setOffsetTransform)
        .def("sigOffsetTransformChanged", &PointSetItem::sigOffsetTransformChanged)
        .def("notifyOffsetTransformChange", &PointSetItem::notifyOffsetTransformChange)
        .def("numAttentionPoints", &PointSetItem::numAttentionPoints)
        .def("attentionPoint", [](PointSetItem& self, int index) {
            return Vector3(self.attentionPoint(index));
        })
        .def("clearAttentionPoints", &PointSetItem::clearAttentionPoints)
        .def("addAttentionPoint", &PointSetItem::addAttentionPoint)
        .def("sigAttentionPointsChanged", &PointSetItem::sigAttentionPointsChanged)
        .def("notifyAttentionPointChange", &PointSetItem::notifyAttentionPointChange)
        ;

    PyItemList<PointSetItem>(m, "PointSetItemList");

    py::class_< MultiPointSetItem, MultiPointSetItemPtr, Item >(m, "MultiPointSetItem")
        .def(py::init<>())
        .def("numPointSetItems", &MultiPointSetItem::numPointSetItems)
        .def("pointSetItem", [](MultiPointSetItem& self, int index) { return PointSetItemPtr(self.pointSetItem(index)); })
        .def("numActivePointSetItems", &MultiPointSetItem::numActivePointSetItems)
        .def("activePointSetItem", [](MultiPointSetItem& self, int index) {
            return PointSetItemPtr(self.activePointSetItem(index));
        })
        .def("sigPointSetItemAdded", &MultiPointSetItem::sigPointSetItemAdded)
        .def("sigPointSetUpdated", &MultiPointSetItem::sigPointSetUpdated)
        .def("topOffsetTransform", &MultiPointSetItem::topOffsetTransform, py::return_value_policy::reference_internal)
        .def("setTopOffsetTransform", &MultiPointSetItem::setTopOffsetTransform)
        .def("sigTopOffsetTransformChanged", &MultiPointSetItem::sigTopOffsetTransformChanged)
        .def("notifyTopOffsetTransformChange", &MultiPointSetItem::notifyTopOffsetTransformChange)
        .def("offsetTransform", &MultiPointSetItem::offsetTransform)
        .def("getTransformedPointSet", &MultiPointSetItem::getTransformedPointSet)
        .def("numAttentionPoints", &MultiPointSetItem::numAttentionPoints)
        .def("attentionPoint", &MultiPointSetItem::attentionPoint)
        .def("clearAttentionPoints", &MultiPointSetItem::clearAttentionPoints)
        .def("addAttentionPoint", &MultiPointSetItem::addAttentionPoint)
        .def("sigAttentionPointsChanged", &MultiPointSetItem::sigAttentionPointsChanged)
        .def("notifyAttentionPointChange", &MultiPointSetItem::notifyAttentionPointChange)
        .def("startAutomaticSave", &MultiPointSetItem::startAutomaticSave)
        .def("stopAutomaticSave", &MultiPointSetItem::stopAutomaticSave);
        ;

    PyItemList<MultiPointSetItem>(m, "MultiPointSetItemList");

#ifdef _MSC_VER
    register_ptr_to_python<ItemPtr>();
	register_ptr_to_python<RootItemPtr>();
#endif

}

} // namespace cnoid

