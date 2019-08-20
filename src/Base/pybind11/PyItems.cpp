/*!
  @author Shin'ichiro Nakaoka
*/

#include "PyItemList.h"
#include "PyQString.h"
#include "../Item.h"
#include "../RootItem.h"
#include "../FolderItem.h"
#include "../SubProjectItem.h"
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
        .def_property("name", &Item::name, &Item::setName)
        .def("setName", &Item::setName)
        .def("hasAttribute", &Item::hasAttribute)
        .def_property_readonly("childItem", &Item::childItem)
        .def_property_readonly("prevItem", &Item::prevItem)
        .def_property_readonly("nextItem", &Item::nextItem)
        .def_property_readonly("parentItem", &Item::parentItem)
        .def("addChildItem", [](Item& self, Item* item){ return self.addChildItem(item); })
        .def("addChildItem", [](Item& self, Item* item, bool isManualOperation){ return self.addChildItem(item, isManualOperation); })
        .def("addSubItem", &Item::addSubItem)
        .def("isSubItem", &Item::isSubItem)
        .def("detachFromParentItem", &Item::detachFromParentItem)
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
        .def_property_readonly("headItem", &Item::headItem)
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
        .def_property_readonly("filePath", &Item::filePath)
        .def_property_readonly("fileFormat", &Item::fileFormat)
        .def("clearFileInformation", &Item::clearFileInformation)
        .def("suggestFileUpdate", &Item::suggestFileUpdate)
        .def("notifyUpdate", &Item::notifyUpdate)
        .def_property_readonly("sigNameChanged", &Item::sigNameChanged)
        .def_property_readonly("sigUpdated", &Item::sigUpdated)
        .def_property_readonly("sigPositionChanged", &Item::sigPositionChanged)
        .def_property_readonly("sigDisconnectedFromRoot", &Item::sigDisconnectedFromRoot)
        .def_property_readonly("sigSubTreeChanged", &Item::sigSubTreeChanged)

        // deprecated
        .def("getName", &Item::name)
        .def("getChildItem", &Item::childItem)
        .def("getPrevItem", &Item::prevItem)
        .def("getNextItem", &Item::nextItem)
        .def("getParentItem", &Item::parentItem)
        .def("getHeadItem", &Item::headItem)
        .def("getFilePath", &Item::filePath)
        .def("getFileFormat", &Item::fileFormat)
        .def("getSigNameChanged", &Item::sigNameChanged)
        .def("getSigUpdated", &Item::sigUpdated)
        .def("getSigPositionChanged", &Item::sigPositionChanged)
        .def("getSigDisconnectedFromRoot", &Item::sigDisconnectedFromRoot)
        .def("getSigSubTreeChanged", &Item::sigSubTreeChanged)
        ;

    py::enum_<Item::Attribute>(itemClass, "Attribute")
        .value("SUB_ITEM", Item::Attribute::SUB_ITEM)
        .value("TEMPORAL", Item::Attribute::TEMPORAL)
        .value("LOAD_ONLY", Item::Attribute::LOAD_ONLY)
        .value("NUM_ATTRIBUTES", Item::Attribute::NUM_ATTRIBUTES)
        .export_values();

    PyItemList<Item>(m, "ItemList");

    py::class_<RootItem, RootItemPtr, Item>(m, "RootItem")
        .def_property_readonly_static("instance", [](py::object){ return RootItem::instance(); })

        // deprecated
        .def_static("getInstance", &RootItem::instance)
        ;

    PyItemList<RootItem>(m, "RootItemList");

    py::class_<FolderItem, FolderItemPtr, Item>(m, "FolderItem")
        .def(py::init<>());

    PyItemList<FolderItem>(m, "FolderItemList");

    py::class_<SubProjectItem, SubProjectItemPtr, Item>(m, "SubProjectItem")
        .def(py::init<>());

    PyItemList<SubProjectItem>(m, "SubProjectItemList");
    
    py::class_<AbstractTextItem, AbstractTextItemPtr, Item>(m, "AbstractTextItem")
        .def_property_readonly("textFilename", &AbstractTextItem::textFilename)

        // deprecated
        .def("getTextFilename", &AbstractTextItem::textFilename)
        ;
            
    //PyItemList<AbstractTextItem>("AbstractTextItemList");
    
    py::class_<ScriptItem, ScriptItemPtr, AbstractTextItem> (m, "ScriptItem")
        .def_property_readonly("scriptFilename", &ScriptItem::scriptFilename)
        .def_property_readonly("identityName", &ScriptItem::identityName)
        .def("setBackgroundMode", &ScriptItem::setBackgroundMode)
        .def("isBackgroundMode", &ScriptItem::isBackgroundMode)
        .def("isRunning", &ScriptItem::isRunning)
        .def("execute", &ScriptItem::execute)
        .def("waitToFinish", [](ScriptItem& self){ return self.waitToFinish(); })
        .def("waitToFinish", [](ScriptItem& self, double timeout){ return self.waitToFinish(timeout); })
        .def_property_readonly("resultString", &ScriptItem::resultString)
        .def_property_readonly("sigScriptFinished", &ScriptItem::sigScriptFinished)
        .def("terminate", &ScriptItem::terminate)

        // deprecated
        .def("getScriptFilename", &ScriptItem::scriptFilename)
        .def("getIdentityName", &ScriptItem::identityName)
        .def("getResultString", &ScriptItem::resultString)
        .def("getSigScriptFinished", &ScriptItem::sigScriptFinished)
        ;

    //PyItemList<ScriptItem>("ScriptItemList");

    py::class_<ExtCommandItem, ExtCommandItemPtr, Item>(m, "ExtCommandItem")
        .def(py::init<>())
        .def_property("command", &ExtCommandItem::command, &ExtCommandItem::setCommand, py::return_value_policy::reference)
        .def("setCommand", &ExtCommandItem::setCommand)
        .def("waitingTimeAfterStarted", &ExtCommandItem::waitingTimeAfterStarted)
        .def("setWaitingTimeAfterStarted", &ExtCommandItem::setWaitingTimeAfterStarted)
        .def("execute", &ExtCommandItem::execute)
        .def("terminate", &ExtCommandItem::terminate)

        // deprecated
        .def("getCommand", &ExtCommandItem::command, py::return_value_policy::reference)
        ;
    
    PyItemList<ExtCommandItem>(m, "ExtCommandItemList");

    // seq items
    py::class_<AbstractSeqItem, AbstractSeqItemPtr, Item> abstractSeqItemClass(m, "AbstractSeqItem");
    abstractSeqItemClass
        .def_property_readonly("abstractSeq", &AbstractSeqItem::abstractSeq)

        // deprecated
        .def("getAbstractSeq", &AbstractSeqItem::abstractSeq)
        ;

    PyItemList<AbstractSeqItem>(m, "AbstractSeqItemList", abstractSeqItemClass);

    py::class_<Vector3SeqItem, Vector3SeqItemPtr, AbstractSeqItem>(m, "Vector3SeqItem")
        .def(py::init<>())
        .def_property_readonly("seq", &Vector3SeqItem::seq)

        // deprecated
        .def("getSeq", &Vector3SeqItem::seq)
        ;

    PyItemList<Vector3SeqItem>(m, "Vector3SeqItemList");

    // multi seq items
    py::class_<AbstractMultiSeqItem, AbstractMultiSeqItemPtr, AbstractSeqItem>
        abstractMultiSeqItemClass(m, "AbstractMultiSeqItem");
    abstractMultiSeqItemClass
        .def_property_readonly("abstractMultiSeq", &AbstractMultiSeqItem::abstractMultiSeq)

        // deprecated
        .def("getAbstractMultiSeq", &AbstractMultiSeqItem::abstractMultiSeq)
        ;

    //PyItemList<AbstractMultiSeqItem>("AbstractMultiSeqItemList", abstractMultiSeqItemClass);
    
    py::class_<MultiValueSeqItem, MultiValueSeqItemPtr, AbstractMultiSeqItem>(m, "MultiValueSeqItem")
        .def(py::init<>())
        .def_property_readonly("seq", &MultiValueSeqItem::seq)

// deprecated
        .def("getSeq", &MultiValueSeqItem::seq);
        ;

    PyItemList<MultiValueSeqItem>(m, "MultiValueSeqItemList");

    py::class_<MultiSE3MatrixSeqItem, MultiSE3MatrixSeqItemPtr, AbstractMultiSeqItem>(m, "MultiSE3MatrixSeqItem")
        .def(py::init<>())
        .def_property_readonly("seq", &MultiSE3MatrixSeqItem::seq)

        // deprecated
        .def("getSeq", &MultiSE3MatrixSeqItem::seq)
        ;

    PyItemList<MultiSE3MatrixSeqItem>(m, "MultiSE3MatrixSeqItemList");

    py::class_<MultiSE3SeqItem, MultiSE3SeqItemPtr, AbstractMultiSeqItem> (m, "MultiSE3SeqItem")
        .def(py::init<>())
        .def_property_readonly("seq", &MultiSE3SeqItem::seq)

        // deprecated
        .def("getSeq", &MultiSE3SeqItem::seq)
        ;
    
    PyItemList<MultiSE3SeqItem>(m, "MultiSE3SeqItemList");

    py::class_<SceneItem, SceneItemPtr, Item>(m, "SceneItem", py::multiple_inheritance())
        .def(py::init<>())
        .def_property_readonly("topNode", (SgPosTransform*(SceneItem::*)()) &SceneItem::topNode)
        .def("setTranslation", &SceneItem::setTranslation)
        .def("setRotation", &SceneItem::setRotation)
        .def("setLightweightRenderingEnabled", &SceneItem::setLightweightRenderingEnabled)
        .def("isLightweightRenderingEnabled", &SceneItem::isLightweightRenderingEnabled)

        // deprecated
        .def("getTopNode", (SgPosTransform*(SceneItem::*)()) &SceneItem::topNode)
        ;

    py::class_<PointSetItem, PointSetItemPtr, Item> (m, "PointSetItem", py::multiple_inheritance())
        .def(py::init<>())
        .def_property("offsetTransform", &PointSetItem::offsetTransform, &PointSetItem::setOffsetTransform)
        .def("setOffsetTransform", &PointSetItem::setOffsetTransform)
        .def_property_readonly("sigOffsetTransformChanged", &PointSetItem::sigOffsetTransformChanged)
        .def("notifyOffsetTransformChange", &PointSetItem::notifyOffsetTransformChange)
        .def_property_readonly("numAttentionPoints", &PointSetItem::numAttentionPoints)
        .def_property_readonly("attentionPoint", (Vector3(PointSetItem::*)(int)const) &PointSetItem::attentionPoint)
        .def("clearAttentionPoints", &PointSetItem::clearAttentionPoints)
        .def("addAttentionPoint", &PointSetItem::addAttentionPoint)
        .def_property_readonly("sigAttentionPointsChanged", &PointSetItem::sigAttentionPointsChanged)
        .def("notifyAttentionPointChange", &PointSetItem::notifyAttentionPointChange)

        // deprecated
        .def("getOffsetTransform", &PointSetItem::offsetTransform)
        .def("getSigOffsetTransformChanged", &PointSetItem::sigOffsetTransformChanged)
        .def("getNumAttentionPoints", &PointSetItem::numAttentionPoints)
        .def("getAttentionPoint", (Vector3(PointSetItem::*)(int)const) &PointSetItem::attentionPoint)
        .def("getSigAttentionPointsChanged", &PointSetItem::sigAttentionPointsChanged)
        ;

    PyItemList<PointSetItem>(m, "PointSetItemList");

    py::class_<MultiPointSetItem, MultiPointSetItemPtr, Item>(m, "MultiPointSetItem", py::multiple_inheritance())
        .def(py::init<>())
        .def_property_readonly("numPointSetItems", &MultiPointSetItem::numPointSetItems)
        .def_property_readonly("pointSetItem", (PointSetItem*(MultiPointSetItem::*)(int)) &MultiPointSetItem::pointSetItem)
        .def_property_readonly("numActivePointSetItems", &MultiPointSetItem::numActivePointSetItems)
        .def_property_readonly("activePointSetItem", (PointSetItem*(MultiPointSetItem::*)(int)) &MultiPointSetItem::activePointSetItem)
        .def_property_readonly("sigPointSetItemAdded", &MultiPointSetItem::sigPointSetItemAdded)
        .def_property_readonly("sigPointSetUpdated", &MultiPointSetItem::sigPointSetUpdated)
        .def_property("topOffsetTransform", &MultiPointSetItem::topOffsetTransform, &MultiPointSetItem::setTopOffsetTransform)
        .def("setTopOffsetTransform", &MultiPointSetItem::setTopOffsetTransform)
        .def_property_readonly("sigTopOffsetTransformChanged", &MultiPointSetItem::sigTopOffsetTransformChanged)
        .def("notifyTopOffsetTransformChange", &MultiPointSetItem::notifyTopOffsetTransformChange)
        .def_property_readonly("offsetTransform", &MultiPointSetItem::offsetTransform)
        .def("getTransformedPointSet", &MultiPointSetItem::getTransformedPointSet)
        .def_property_readonly("numAttentionPoints", &MultiPointSetItem::numAttentionPoints)
        .def("attentionPoint", &MultiPointSetItem::attentionPoint)
        .def("clearAttentionPoints", &MultiPointSetItem::clearAttentionPoints)
        .def("addAttentionPoint", &MultiPointSetItem::addAttentionPoint)
        .def_property_readonly("sigAttentionPointsChanged", &MultiPointSetItem::sigAttentionPointsChanged)
        .def("notifyAttentionPointChange", &MultiPointSetItem::notifyAttentionPointChange)
        .def("startAutomaticSave", &MultiPointSetItem::startAutomaticSave)
        .def("stopAutomaticSave", &MultiPointSetItem::stopAutomaticSave)

        // deprecated
        .def("getNumPointSetItems", &MultiPointSetItem::numPointSetItems)
        .def("getPointSetItem", (PointSetItem*(MultiPointSetItem::*)(int)) &MultiPointSetItem::pointSetItem)
        .def("getNumActivePointSetItems", &MultiPointSetItem::numActivePointSetItems)
        .def("getActivePointSetItem", (PointSetItem*(MultiPointSetItem::*)(int)) &MultiPointSetItem::activePointSetItem)
        .def("getSigPointSetItemAdded", &MultiPointSetItem::sigPointSetItemAdded)
        .def("getSigPointSetUpdated", &MultiPointSetItem::sigPointSetUpdated)
        .def("getTopOffsetTransform", &MultiPointSetItem::topOffsetTransform)
        .def("getSigTopOffsetTransformChanged", &MultiPointSetItem::sigTopOffsetTransformChanged)
        .def("getOffsetTransform", &MultiPointSetItem::offsetTransform)
        .def("getNumAttentionPoints", &MultiPointSetItem::numAttentionPoints)
        .def("getAttentionPoint", &MultiPointSetItem::attentionPoint)
        .def("getSigAttentionPointsChanged", &MultiPointSetItem::sigAttentionPointsChanged)
        ;

    PyItemList<MultiPointSetItem>(m, "MultiPointSetItemList");
}

} // namespace cnoid
