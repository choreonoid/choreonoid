/*!
  @author Shin'ichiro Nakaoka
*/

#include "PyItemList.h"
#include "PyQString.h"
#include "../Item.h"
#include "../RootItem.h"
#include "../FolderItem.h"
#include "../AbstractTextItem.h"
#include "../ScriptItem.h"
#include "../ExtCommandItem.h"
#include "../MultiValueSeqItem.h"
#include "../MultiSE3SeqItem.h"
#include "../MultiSE3MatrixSeqItem.h"
#include "../Vector3SeqItem.h"
#include "../SceneItem.h"
#include "../PointSetItem.h"
#include "../MultiPointSetItem.h"
#include <cnoid/PyReferenced>
#include <cnoid/PyEigenTypes>

using namespace std;
using namespace cnoid;
namespace py = pybind11;

namespace cnoid {

void exportPyItems(py::module m)
{
    py::class_<Item, ItemPtr, Referenced> itemClass(m, "Item");

    itemClass
        .def_static("find", [](const string& path){ return ItemPtr(Item::find(path)); })
        .def("name", &Item::name)
        .def("setName", &Item::setName)
        .def("hasAttribute", &Item::hasAttribute)
        .def("childItem", &Item::childItem)
        .def("prevItem", &Item::prevItem)
        .def("nextItem", &Item::nextItem)
        .def("parentItem", &Item::parentItem)
        .def("addChildItem", [](Item& self, Item* item){ return self.addChildItem(item); })
        .def("addChildItem", [](Item& self, Item* item, bool isManualOperation){ return self.addChildItem(item, isManualOperation); })
        .def("addSubItem", &Item::addSubItem)
        .def("isSubItem", &Item::isSubItem)
        .def("detachFromParentItem", &Item::detachFromParentItem)
        .def("emitSigDetachedFromRootForSubTree", &Item::emitSigDetachedFromRootForSubTree)
        .def("insertChildItem", [](Item& self, Item* item, Item* nextItem){ return self.insertChildItem(item, nextItem); })
        .def("insertChildItem", [](Item& self, Item* item, Item* nextItem, bool isManualOperation){
                return self.insertChildItem(item, nextItem, isManualOperation); })
        .def("insertSubItem", &Item::insertSubItem)
        .def("isTemporal", &Item::isTemporal)
        .def("setTemporal", [](Item& self){ self.setTemporal(); })
        .def("setTemporal", [](Item& self, bool on){ self.setTemporal(on); })
        .def("findRootItem", &Item::findRootItem)
        .def("findItem", [](Item& self, const string& path){ return self.findItem(path); })
        .def("findChildItem", [](Item& self, const string& path){ return self.findChildItem(path); })
        .def("findSubItem", [](Item& self, const string& path){ return self.findSubItem(path); })
        .def("headItem", &Item::headItem)
        .def("getDescendantItems", [](Item& self){ ItemList<Item> items; items.extractChildItems(&self); return items; })
        .def("getDescendantItems", [](Item& self, py::object itemClass) {
            ItemList<Item> items; items.extractChildItems(&self); return getPyNarrowedItemList(items, itemClass); })
        .def("duplicate", &Item::duplicate)
        .def("duplicateAll", &Item::duplicateAll)
        .def("assign", &Item::assign)
        .def("load", [](Item& self, const string& filename){ return self.load(filename); })
        .def("load", [](Item& self, const string& filename, const string& format){ return self.load(filename, format); })
        .def("load", [](Item& self, Item* parent, const string& filename){ return self.load(filename, parent); })
        .def("load", [](Item& self, Item* parent, const string& filename, const string& format){ return self.load(filename, parent, format); })
        .def("save", [](Item& self, const string& filename){ return self.save(filename); })
        .def("save", [](Item& self, const string& filename, const string& format){ return self.save(filename, format); })
        .def("overwrite",[](Item& self){ return self.overwrite(); })
        .def("overwrite",[](Item& self, bool forceOverwrite){ return self.overwrite(forceOverwrite); })
        .def("overwrite",[](Item& self, bool forceOverwrite, const string& format){ return self.overwrite(forceOverwrite, format); })
        .def("filePath", &Item::filePath)
        .def("fileFormat", &Item::fileFormat)
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

    py::class_<RootItem, RootItemPtr, Item>(m, "RootItem")
        .def_static("instance", &RootItem::instance);

    PyItemList<RootItem>(m, "RootItemList");

    py::class_<FolderItem, FolderItemPtr, Item>(m, "FolderItem")
        .def(py::init<>());

    PyItemList<FolderItem>(m, "FolderItemList");

    py::class_<AbstractTextItem, AbstractTextItemPtr, Item>(m, "AbstractTextItem")
        .def("textFilename", &AbstractTextItem::textFilename);
            
    //PyItemList<AbstractTextItem>("AbstractTextItemList");
    
    py::class_<ScriptItem, ScriptItemPtr, AbstractTextItem> (m, "ScriptItem")
        .def("scriptFilename", &ScriptItem::scriptFilename)
        .def("identityName", &ScriptItem::identityName)
        .def("setBackgroundMode", &ScriptItem::setBackgroundMode)
        .def("isBackgroundMode", &ScriptItem::isBackgroundMode)
        .def("isRunning", &ScriptItem::isRunning)
        .def("execute", &ScriptItem::execute)
        .def("waitToFinish", [](ScriptItem& self){ return self.waitToFinish(); })
        .def("waitToFinish", [](ScriptItem& self, double timeout){ return self.waitToFinish(timeout); })
        .def("resultString", &ScriptItem::resultString)
        .def("sigScriptFinished", &ScriptItem::sigScriptFinished)
        .def("terminate", &ScriptItem::terminate)
        ;

    //PyItemList<ScriptItem>("ScriptItemList");

    py::class_<ExtCommandItem, ExtCommandItemPtr, Item>(m, "ExtCommandItem")
        .def(py::init<>())
        .def("setCommand", &ExtCommandItem::setCommand)
        .def("command", &ExtCommandItem::command, py::return_value_policy::reference)
        .def("waitingTimeAfterStarted", &ExtCommandItem::waitingTimeAfterStarted)
        .def("setWaitingTimeAfterStarted", &ExtCommandItem::setWaitingTimeAfterStarted)
        .def("execute", &ExtCommandItem::execute)
        .def("terminate", &ExtCommandItem::terminate);
    
    PyItemList<ExtCommandItem>(m, "ExtCommandItemList");

    // seq items
    py::class_<AbstractSeqItem, AbstractSeqItemPtr, Item> abstractSeqItemClass(m, "AbstractSeqItem");
    abstractSeqItemClass
        .def("abstractSeq", &AbstractSeqItem::abstractSeq);

    PyItemList<AbstractSeqItem>(m, "AbstractSeqItemList", abstractSeqItemClass);

    py::class_<Vector3SeqItem, Vector3SeqItemPtr, AbstractSeqItem>(m, "Vector3SeqItem")
        .def(py::init<>())
        .def("seq", &Vector3SeqItem::seq);

    PyItemList<Vector3SeqItem>(m, "Vector3SeqItemList");

    // multi seq items
    py::class_<AbstractMultiSeqItem, AbstractMultiSeqItemPtr, AbstractSeqItem>
        abstractMultiSeqItemClass(m, "AbstractMultiSeqItem");
    abstractMultiSeqItemClass
        .def("abstractMultiSeq", &AbstractMultiSeqItem::abstractMultiSeq);

    //PyItemList<AbstractMultiSeqItem>("AbstractMultiSeqItemList", abstractMultiSeqItemClass);
    
    py::class_<MultiValueSeqItem, MultiValueSeqItemPtr, AbstractMultiSeqItem>(m, "MultiValueSeqItem")
        .def(py::init<>())
        .def("abstractMultiSeq", &MultiValueSeqItem::abstractMultiSeq)
        .def("seq", &MultiValueSeqItem::seq);

    PyItemList<MultiValueSeqItem>(m, "MultiValueSeqItemList");

    py::class_<MultiSE3MatrixSeqItem, MultiSE3MatrixSeqItemPtr, AbstractMultiSeqItem>(m, "MultiSE3MatrixSeqItem")
        .def(py::init<>())
        .def("abstractMultiSeq", &MultiSE3MatrixSeqItem::abstractMultiSeq)
        .def("seq", &MultiSE3MatrixSeqItem::seq);

    PyItemList<MultiSE3MatrixSeqItem>(m, "MultiSE3MatrixSeqItemList");

    py::class_<MultiSE3SeqItem, MultiSE3SeqItemPtr, AbstractMultiSeqItem> (m, "MultiSE3SeqItem")
        .def(py::init<>())
        .def("abstractMultiSeq", &MultiSE3SeqItem::abstractMultiSeq)
        .def("seq", &MultiSE3SeqItem::seq);
    
    PyItemList<MultiSE3SeqItem>(m, "MultiSE3SeqItemList");

    py::class_<SceneItem, SceneItemPtr, Item>(m, "SceneItem", py::multiple_inheritance())
        .def(py::init<>())
        .def("topNode", (SgPosTransform*(SceneItem::*)()) &SceneItem::topNode)
        ;

    py::class_<PointSetItem, PointSetItemPtr, Item> (m, "PointSetItem", py::multiple_inheritance())
        .def(py::init<>())
        .def("offsetTransform", &PointSetItem::offsetTransform)
        .def("setOffsetTransform", &PointSetItem::setOffsetTransform)
        .def("sigOffsetTransformChanged", &PointSetItem::sigOffsetTransformChanged)
        .def("notifyOffsetTransformChange", &PointSetItem::notifyOffsetTransformChange)
        .def("numAttentionPoints", &PointSetItem::numAttentionPoints)
        .def("attentionPoint", (Vector3(PointSetItem::*)(int)const) &PointSetItem::attentionPoint)
        .def("clearAttentionPoints", &PointSetItem::clearAttentionPoints)
        .def("addAttentionPoint", &PointSetItem::addAttentionPoint)
        .def("sigAttentionPointsChanged", &PointSetItem::sigAttentionPointsChanged)
        .def("notifyAttentionPointChange", &PointSetItem::notifyAttentionPointChange)
        ;

    PyItemList<PointSetItem>(m, "PointSetItemList");

    py::class_<MultiPointSetItem, MultiPointSetItemPtr, Item>(m, "MultiPointSetItem", py::multiple_inheritance())
        .def(py::init<>())
        .def("numPointSetItems", &MultiPointSetItem::numPointSetItems)
        .def("pointSetItem", (PointSetItem*(MultiPointSetItem::*)(int)) &MultiPointSetItem::pointSetItem)
        .def("numActivePointSetItems", &MultiPointSetItem::numActivePointSetItems)
        .def("activePointSetItem", (PointSetItem*(MultiPointSetItem::*)(int)) &MultiPointSetItem::activePointSetItem)
        .def("sigPointSetItemAdded", &MultiPointSetItem::sigPointSetItemAdded)
        .def("sigPointSetUpdated", &MultiPointSetItem::sigPointSetUpdated)
        .def("topOffsetTransform", &MultiPointSetItem::topOffsetTransform)
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
}

} // namespace cnoid
